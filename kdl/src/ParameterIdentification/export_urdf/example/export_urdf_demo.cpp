// Copyright (c) 2026, kdl_identification authors.
// 教学示例：把辨识出的参数导出成 URDF，并用它验证参数辨识的作用。
//
// ===========================================================================
// 一、为什么需要一个"真值 ≠ 先验"的场景
// ===========================================================================
// 如果直接在本仓库的仿真里辨识，会得到一个尴尬的结论：仿真真值就是 URDF
// 本身，辨识出来的模型必然等于先验模型，"辨识有什么用"根本看不出来。
//
// 所以本示例构造一个**离线、可复现**的场景：
//
//   真值模型  = 对 URDF 的惯量做"物理可实现"的扰动（模拟真机与 CAD 不一致）
//   先验模型  = 原 URDF（控制器/仿真里相信的那份，即"错的"那份）
//   实测数据  = 用真值模型在傅里叶激励轨迹上的逆动力学力矩（可选加噪声）
//
// 然后走完整流程：辨识 → 零空间补全 → 导出新 URDF → 在**留出轨迹**上比较
// "先验模型"与"辨识模型"的力矩预测误差。这才体现得出辨识的作用。
//
// ===========================================================================
// 二、为什么"导出"必须配"零空间补全"
// ===========================================================================
// 最小二乘会把不可辨识的参数置 0（见 kdl_lsq）。串联臂里"第一连杆的质量"
// 通常结构性不可辨识，于是 β_id 出现 m = 0 而 m·cx ≠ 0 —— URDF 与 KDL 都
// 无法表示（KDL 里 h 恒等于 m·c），直接导出必然失败。
// completeWithPrior() 在所有等价解里挑最接近 URDF 先验的那个，只改动对任何
// 运动都不影响 τ 的零空间方向，于是参数重新变得物理可实现。
//
// ===========================================================================
// 三、运行
// ===========================================================================
//   colcon build --paths .
//   source install/setup.bash
//   ros2 run kdl_tools export_urdf_demo
//   ros2 run kdl_tools export_urdf_demo --noise 0.01     # 加 0.01 N·m 测量噪声
//
// 产物：<export_urdf>/robotic_arm_identified.urdf（可直接喂给控制器的
//       urdf_file 参数，或 robot_state_publisher）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

#include "kdl_fourier.hpp"
#include "kdl_idynamics.hpp"
#include "kdl_lmi_projection.hpp"
#include "kdl_lsq.hpp"
#include "kdl_regressor.hpp"
#include "kdl_tools.hpp"
#include "kdl_urdf_export.hpp"

namespace
{

#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

#ifndef KDL_EXPORT_URDF_DIR
#define KDL_EXPORT_URDF_DIR "."
#endif

using kdl_identification::kParamsPerLink;

/// 判定"质量为零"的阈值（kg），与 kdl_regressor / kdl_urdf_export 保持一致。
constexpr double kMassEpsilon = 1e-12;

/// 加速度上限没有写在 URDF 里，用速度上限的这个倍数作为工程近似。
constexpr double kAccelerationOverVelocity = 5.0;

/// 辨识用采样点数（激励轨迹一个周期内等间隔取点）。
constexpr unsigned int kTrainSamples = 200;

/// 零空间补全用的随机采样点数（要覆盖尽量多的运动状态）。
constexpr unsigned int kCompletionSamples = 600;

/// 留出轨迹的采样点数。
constexpr unsigned int kHoldoutSamples = 300;

/// 判定"导出正确"的参数往返阈值。
constexpr double kRoundTripTolerance = 1e-9;

/// 误差统计。
struct ErrorStats
{
  double rms = 0.0;
  double max = 0.0;
};

/**
 * @brief 从 URDF 读关节限位（位置上下限、速度上限；加速度取速度的固定倍数）。
 */
bool readJointLimits(
  const std::string & urdf_file, const KDL::Chain & chain, kdl_identification::FourierLimits & limits,
  std::string & message)
{
  urdf::Model model;
  if (!kdl_tools::loadUrdfModel(urdf_file, model)) {
    message = "无法解析 URDF：" + urdf_file;
    return false;
  }

  const unsigned int n = chain.getNrOfJoints();
  limits.q_min = KDL::JntArray(n);
  limits.q_max = KDL::JntArray(n);
  limits.qdot_max = KDL::JntArray(n);
  limits.qddot_max = KDL::JntArray(n);

  unsigned int index = 0;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // 固定段不占自由度
    }
    const auto it = model.joints_.find(joint.getName());
    if (it == model.joints_.end() || !it->second->limits) {
      message = "关节 " + joint.getName() + " 缺少 <limit>";
      return false;
    }
    limits.q_min(index) = it->second->limits->lower;
    limits.q_max(index) = it->second->limits->upper;
    limits.qdot_max(index) = it->second->limits->velocity;
    limits.qddot_max(index) = kAccelerationOverVelocity * it->second->limits->velocity;
    ++index;
  }
  if (index != n) {
    message = "链上可动关节数（" + std::to_string(index) + "）与自由度（" + std::to_string(n) +
      "）不一致";
    return false;
  }
  message.clear();
  return true;
}

/**
 * @brief 由先验参数造一个"真值"参数（模拟真机与 CAD/URDF 不一致）。
 * @param prior     [in]  先验参数（URDF 原值）。
 * @param segments  [in]  段数。
 * @param seed      [in]  随机种子。
 * @param mass_scale[out] 每个 link 的质量缩放系数（打印用）。
 * @return 扰动后的参数向量。
 *
 * @note 扰动方式是**分别缩放 (m, c, I_cog)**，而不是直接缩放 β 里的 I_ref：
 *       后者的平行轴项与质量不一致，可能把 I_cog 变成非正定（非物理）。
 *       分别缩放时 I_cog' = β·I_cog（β>0 保正定）、平行轴项也保持半正定，
 *       所以扰动后的模型一定物理可实现、也一定能被 URDF 表示。
 */
Eigen::VectorXd makePerturbedParameters(
  const Eigen::VectorXd & prior, unsigned int segments, unsigned int seed,
  std::vector<double> & mass_scale)
{
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> mass_dist(0.75, 1.35);
  std::uniform_real_distribution<double> cog_dist(0.80, 1.25);
  std::uniform_real_distribution<double> inertia_dist(0.70, 1.40);

  Eigen::VectorXd perturbed = prior;
  mass_scale.assign(segments, 1.0);

  for (unsigned int s = 0; s < segments; ++s) {
    const double m = prior[kParamsPerLink * s];
    if (!(m > kMassEpsilon)) {
      continue;  // 退化段（质量 0）保持原样
    }
    const Eigen::Vector3d h(prior[kParamsPerLink * s + 1], prior[kParamsPerLink * s + 2],
      prior[kParamsPerLink * s + 3]);
    Eigen::Matrix3d i_ref;
    i_ref << prior[kParamsPerLink * s + 4], prior[kParamsPerLink * s + 5],
      prior[kParamsPerLink * s + 6], prior[kParamsPerLink * s + 5], prior[kParamsPerLink * s + 7],
      prior[kParamsPerLink * s + 8], prior[kParamsPerLink * s + 6], prior[kParamsPerLink * s + 8],
      prior[kParamsPerLink * s + 9];

    const Eigen::Vector3d cog = h / m;
    const Eigen::Matrix3d i_cog =
      i_ref - m * (cog.dot(cog) * Eigen::Matrix3d::Identity() - cog * cog.transpose());

    const double new_mass = m * mass_dist(generator);
    const Eigen::Vector3d new_cog = cog * cog_dist(generator);
    const Eigen::Matrix3d new_i_cog = i_cog * inertia_dist(generator);
    const Eigen::Matrix3d new_i_ref =
      new_i_cog + new_mass * (new_cog.dot(new_cog) * Eigen::Matrix3d::Identity() -
      new_cog * new_cog.transpose());

    mass_scale[s] = new_mass / m;
    auto block = perturbed.segment<kParamsPerLink>(kParamsPerLink * s);
    block[0] = new_mass;
    block[1] = new_mass * new_cog.x();
    block[2] = new_mass * new_cog.y();
    block[3] = new_mass * new_cog.z();
    block[4] = new_i_ref(0, 0);
    block[5] = new_i_ref(0, 1);
    block[6] = new_i_ref(0, 2);
    block[7] = new_i_ref(1, 1);
    block[8] = new_i_ref(1, 2);
    block[9] = new_i_ref(2, 2);
  }
  return perturbed;
}

/**
 * @brief 用一条链算一批采样点的逆动力学力矩（堆叠成 joints × samples 的向量）。
 */
Eigen::VectorXd torqueOf(const KDL::Chain & chain, const std::vector<kdl_identification::RegressorSample> & samples)
{
  const unsigned int n = chain.getNrOfJoints();
  Eigen::VectorXd torque(static_cast<Eigen::Index>(samples.size() * n));
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const kdl_dynamics::IdResult id =
      kdl_dynamics::inverseDynamics(chain, samples[i].q, samples[i].qdot, samples[i].qddot);
    for (unsigned int j = 0; j < n; ++j) {
      torque(static_cast<Eigen::Index>(i * n + j)) = id.success() ? id.torque(j) : 0.0;
    }
  }
  return torque;
}

/**
 * @brief 在关节限位内随机造一批运动状态。
 * @note 零空间补全的 W 必须覆盖尽量多的运动状态：只用一条激励轨迹会把
 *       "该轨迹上没激励到"误判成"结构性不可辨识"。
 */
std::vector<kdl_identification::RegressorSample> randomSamples(
  const KDL::Chain & chain, const kdl_identification::FourierLimits & limits, unsigned int count,
  unsigned int seed)
{
  const unsigned int n = chain.getNrOfJoints();
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);

  std::vector<kdl_identification::RegressorSample> samples;
  samples.reserve(count);
  for (unsigned int i = 0; i < count; ++i) {
    kdl_identification::RegressorSample sample;
    sample.q = KDL::JntArray(n);
    sample.qdot = KDL::JntArray(n);
    sample.qddot = KDL::JntArray(n);
    for (unsigned int j = 0; j < n; ++j) {
      // 位置：限位内取 90% 行程，避免正好贴在边界上。
      const double middle = 0.5 * (limits.q_min(j) + limits.q_max(j));
      const double half = 0.45 * (limits.q_max(j) - limits.q_min(j));
      sample.q(j) = middle + half * unit(generator);
      sample.qdot(j) = 0.5 * limits.qdot_max(j) * unit(generator);
      sample.qddot(j) = 0.5 * limits.qddot_max(j) * unit(generator);
    }
    samples.push_back(std::move(sample));
  }
  return samples;
}

/// 两组向量的误差统计。
ErrorStats errorStats(const Eigen::VectorXd & actual, const Eigen::VectorXd & reference)
{
  ErrorStats stats;
  const Eigen::VectorXd difference = actual - reference;
  stats.rms = std::sqrt(difference.squaredNorm() / static_cast<double>(difference.size()));
  stats.max = difference.cwiseAbs().maxCoeff();
  return stats;
}

/// 在两组参数里找"相对差异最大"的那一项，返回 (下标, 相对差异)。
std::pair<unsigned int, double> worstParameterDifference(
  const Eigen::VectorXd & a, const Eigen::VectorXd & b)
{
  unsigned int worst_index = 0;
  double worst = -1.0;
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    const double scale = std::max(std::abs(b(i)), 1e-9);
    const double relative = std::abs(a(i) - b(i)) / scale;
    if (relative > worst) {
      worst = relative;
      worst_index = static_cast<unsigned int>(i);
    }
  }
  return {worst_index, worst};
}

}  // namespace

int main(int argc, char ** argv)
{
  double noise = 0.0;
  bool prefer_feasible = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--noise") == 0 && i + 1 < argc) {
      noise = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--feasible") == 0) {
      prefer_feasible = true;
    }
  }

  const std::string urdf_in = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  const std::string urdf_out = std::string(KDL_EXPORT_URDF_DIR) + "/robotic_arm_identified.urdf";

  std::printf("=== 参数辨识 → URDF 导出 → 留出验证 ===\n");
  std::printf("先验 URDF：%s\n", urdf_in.c_str());
  std::printf("输出 URDF：%s\n", urdf_out.c_str());
  std::printf("测量噪声  ：%.4f N·m\n\n", noise);

  std::string message;

  // -----------------------------------------------------------------------
  // 1) 先验模型：URDF 原值。
  // -----------------------------------------------------------------------
  KDL::Chain prior_chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_in, prior_chain)) {
    std::printf("建链失败：%s\n", urdf_in.c_str());
    return 1;
  }
  const unsigned int n = prior_chain.getNrOfJoints();
  const unsigned int segments = prior_chain.getNrOfSegments();

  Eigen::VectorXd beta_prior;
  if (!kdl_identification::extractParameters(prior_chain, beta_prior, message)) {
    std::printf("读取先验参数失败：%s\n", message.c_str());
    return 1;
  }
  std::printf("[1] 先验模型：%u 自由度 / %u 段 / %u 个参数\n", n, segments,
    kdl_identification::parameterCount(prior_chain));

  // -----------------------------------------------------------------------
  // 2) 真值模型：物理可实现的扰动，模拟"真机与 CAD 不一致"。
  // -----------------------------------------------------------------------
  std::vector<double> mass_scale;
  const Eigen::VectorXd beta_true = makePerturbedParameters(beta_prior, segments, 20261006u, mass_scale);
  KDL::Chain true_chain;
  if (!kdl_identification::applyParameters(prior_chain, beta_true, true_chain, message)) {
    std::printf("构造真值模型失败：%s\n", message.c_str());
    return 1;
  }
  std::printf("[2] 真值模型：对每个 link 的质量做 ±25%%、质心 ±20%%、质心惯量 ±30%% 的扰动\n");
  for (unsigned int s = 0; s < segments; ++s) {
    std::printf("      %-7s 质量 ×%.3f\n", prior_chain.getSegment(s).getName().c_str(),
      mass_scale[s]);
  }

  // -----------------------------------------------------------------------
  // 3) 激励轨迹：用**先验模型**的限位与几何（回归矩阵只由几何决定，与惯量无关）。
  // -----------------------------------------------------------------------
  kdl_identification::FourierLimits limits;
  if (!readJointLimits(urdf_in, prior_chain, limits, message)) {
    std::printf("读取关节限位失败：%s\n", message.c_str());
    return 1;
  }

  kdl_identification::ExcitationOptions excitation_options;
  excitation_options.random_seed = 20261005u;
  const kdl_identification::ExcitationResult excitation =
    kdl_identification::optimizeFourierExcitation(prior_chain, limits, excitation_options);
  if (!excitation.success) {
    std::printf("激励轨迹优化失败：%s\n", excitation.message.c_str());
    return 1;
  }
  std::printf(
    "\n[3] 激励轨迹：rank=%u/%u，σ_min⁺=%.3e，cond=%.3e，λ=%.4f\n",
    excitation.observability.rank, excitation.observability.total_parameters,
    excitation.observability.min_singular_value, excitation.observability.condition_number,
    excitation.scale);

  // -----------------------------------------------------------------------
  // 4) "实测"数据：真值模型的力矩（可选加噪声）。
  // -----------------------------------------------------------------------
  std::vector<kdl_identification::RegressorSample> train_samples;
  excitation.trajectory.samplePeriod(kTrainSamples, train_samples);
  Eigen::VectorXd tau_train = torqueOf(true_chain, train_samples);
  if (noise > 0.0) {
    std::mt19937 generator(20261007u);
    std::normal_distribution<double> normal(0.0, noise);
    for (Eigen::Index i = 0; i < tau_train.size(); ++i) {
      tau_train(i) += normal(generator);
    }
  }
  std::printf("[4] 训练数据：%zu 个采样点 × %u 关节 = %zu 行\n", train_samples.size(), n,
    static_cast<std::size_t>(tau_train.size()));

  // -----------------------------------------------------------------------
  // 5) 回归矩阵 + 最小二乘（几何来自先验链，与真值链一致）。
  // -----------------------------------------------------------------------
  kdl_identification::RegressorBuilder builder(prior_chain);
  if (!builder.valid()) {
    std::printf("回归矩阵构造器不可用：%s\n", builder.error().c_str());
    return 1;
  }
  const kdl_identification::RegressorResult train_regressor = builder.build(train_samples);
  if (!train_regressor.success) {
    std::printf("构建回归矩阵失败：%s\n", train_regressor.message.c_str());
    return 1;
  }
  const kdl_identification::IdentificationResult solution =
    kdl_identification::solveLeastSquares(train_regressor.regressor, tau_train);
  if (!solution.success) {
    std::printf("辨识失败：%s\n", solution.message.c_str());
    return 1;
  }
  std::printf(
    "[5] 辨识：数值秩 = %u / %u，样本内残差 RMS = %.3e / MAX = %.3e N·m，cond = %.3e\n",
    solution.rank, solution.total_parameters, solution.residual_rms, solution.residual_max,
    solution.scaled_condition_number);

  // -----------------------------------------------------------------------
  // 6) 零空间补全：把"不可辨识"的分量从先验搬回来，使参数物理可实现。
  //    W 用大范围随机采样，避免把"这条轨迹没激励到"误判成"结构性不可辨识"。
  // -----------------------------------------------------------------------
  const std::vector<kdl_identification::RegressorSample> completion_samples =
    randomSamples(prior_chain, limits, kCompletionSamples, 20261008u);
  const kdl_identification::RegressorResult completion_regressor = builder.build(completion_samples);
  if (!completion_regressor.success) {
    std::printf("构建补全用回归矩阵失败：%s\n", completion_regressor.message.c_str());
    return 1;
  }
  const kdl_identification::CompletionResult completion = kdl_identification::completeWithPrior(
    completion_regressor.regressor, solution.parameters, beta_prior);
  if (!completion.success) {
    std::printf("零空间补全失败：%s\n", completion.message.c_str());
    return 1;
  }
  std::printf(
    "[6] 零空间补全：秩 = %u，零空间维数 = %u\n"
    "      与先验的距离：%.3e（补全前 %.3e，收缩到 %.1f%%）\n",
    completion.rank, completion.null_space_dimension, completion.correction_norm,
    completion.prior_gap_norm,
    completion.prior_gap_norm > 0.0 ? 100.0 * completion.correction_norm / completion.prior_gap_norm : 0.0);

  // 对照实验：如果改用欧氏度量（s_j ≡ 1，即"绝对偏差最小"），会怎样？
  // 目的是把"度量选择不是细节"这件事用数字摆出来。
  const auto worst_cog_norm = [](const Eigen::VectorXd & beta, unsigned int count) {
      double worst = 0.0;
      for (unsigned int s = 0; s < count; ++s) {
        const double m = beta[kParamsPerLink * s];
        if (!(m > kMassEpsilon)) {
          continue;
        }
        const Eigen::Vector3d h(beta[kParamsPerLink * s + 1], beta[kParamsPerLink * s + 2],
          beta[kParamsPerLink * s + 3]);
        worst = std::max(worst, h.norm() / m);
      }
      return worst;
    };
  {
    kdl_identification::CompletionOptions euclidean_options;
    euclidean_options.scale = Eigen::VectorXd::Ones(completion_regressor.regressor.cols());
    const kdl_identification::CompletionResult euclidean =
      kdl_identification::completeWithPrior(
      completion_regressor.regressor, solution.parameters, beta_prior, euclidean_options);
    std::printf(
      "      [对照] 最大 |质心|：欧氏度量 %.3f m，本度量 %.3f m，先验本身 %.3f m\n"
      "             （零空间含\"沿关节轴平移质心\"这类方向：对 τ 无影响，但会把质心推远）\n",
      worst_cog_norm(euclidean.parameters, segments),
      worst_cog_norm(completion.parameters, segments),
      worst_cog_norm(beta_prior, segments));
  }

  // 补全前不可表示、补全后应可表示 —— 这是"必须补全"的直接证据。
  std::string before_message;
  const bool representable_before =
    kdl_identification::parametersRepresentableInUrdf(prior_chain, solution.parameters, before_message);
  std::printf("      补全前可直接导出？%s", representable_before ? "是" : "否");
  if (!representable_before) {
    std::printf("  ← %s", before_message.c_str());
  }
  std::printf("\n");

  // 物理可行性：零空间补全只保证 W·β = W·β_id，**不保证** β 物理可行。
  // 可行集是伪惯量矩阵 J = [[½tr(I_ref)Id − I_ref, h], [hᵀ, m]] ≻ 0（一个 LMI），
  // 与"τ 等价的仿射集"
  // 并不重合，所以这里必须显式检查；不可行时再走"可行集 ↔ 仿射集"交替投影。
  const kdl_identification::PhysicalFeasibility raw_feasibility =
    kdl_identification::checkPhysicalFeasibility(completion.parameters);
  std::printf("      物理性检查（未加约束）：%s", raw_feasibility.feasible ? "通过" : "**未通过**");
  if (!raw_feasibility.feasible) {
    std::printf("  ← 段 %s（%s）",
      prior_chain.getSegment(raw_feasibility.worst_segment).getName().c_str(),
      raw_feasibility.message.c_str());
  }
  std::printf("\n");

  // -----------------------------------------------------------------------
  // 6b) LMI 约束求解（正解）：min ‖S(β−β_prior)‖ s.t. Wβ = Wβ_id, J_s(β) ⪰ 0
  //     上面那套"投影 + 退让"是启发式，只能退到先验；这一步才是在可行域**内部**
  //     找最优点。两个集合都凸，所以这是标准 SDP，本模块自带小规模求解器。
  // -----------------------------------------------------------------------
  kdl_identification::CompletionOptions scale_options;  // 复用同一套度量尺度
  const kdl_identification::LmiSolveResult lmi = kdl_identification::solveLmiConstrainedCompletion(
    completion_regressor.regressor, solution.parameters, beta_prior,
    scale_options.scale, 1e-9);
  std::printf(
    "\n      [6b] LMI 约束求解：%s\n", lmi.success ? "成功" : "失败");
  if (lmi.success) {
    std::printf(
      "        自由变量 %u，外层 %u 轮 / Newton %u 步，末次移位 σ = %.3e\n"
      "        伪惯量最小特征值 = %.6e（> 0 即严格可行）\n"
      "        目标值 ‖S(β−β_prior)‖ = %.6e（对比：退让方案 %.6e）\n",
      lmi.variables, lmi.outer_iterations, lmi.newton_steps, lmi.final_shift,
      lmi.min_eigenvalue, lmi.scaled_distance, completion.correction_norm);
  } else {
    std::printf("        %s\n", lmi.message.c_str());
  }

  // 实际导出的参数：默认用**力矩拟合最好**的那份；--feasible 时改用物理可行的那份。
  // 两者存在真实取舍，见下面打印的代价。
  Eigen::VectorXd beta_export = completion.parameters;
  bool exported_is_feasible = raw_feasibility.feasible;
  if (lmi.success) {
    // LMI 解**同时**满足两个条件，所以直接用它，不需要任何取舍。
    beta_export = lmi.parameters;
    exported_is_feasible = true;
    std::printf("        采用 LMI 解导出（力矩与可行性同时满足，无需取舍）\n");
  } else if (!raw_feasibility.feasible) {
    kdl_identification::CompletionOptions guarded_options;
    guarded_options.enforce_physical_feasibility = true;
    const kdl_identification::CompletionResult guarded = kdl_identification::completeWithPrior(
      completion_regressor.regressor, solution.parameters, beta_prior, guarded_options);
    if (!guarded.success) {
      std::printf("      可行性投影失败：%s\n", guarded.message.c_str());
    } else {
      std::printf(
        "      可行性投影（可行集 ↔ 仿射集 交替投影 + 线段退让）：%s\n"
        "        采用比例 t = %.4f，力矩偏差 %.3e N·m（相对辨识解）\n"
        "        t = 1 才表示落在两集交集里（力矩与可行性同时满足）；本例 t≈0，\n"
        "        即退回了先验 —— 说明两个集合在\"先验附近\"几乎不相交。\n",
        guarded.physically_feasible ? "得到可行解" : "未得到可行解",
        guarded.feasibility_fraction, guarded.torque_residual_norm);
      if (prefer_feasible && guarded.physically_feasible) {
        beta_export = guarded.parameters;
        exported_is_feasible = true;
      }
    }
  }
  std::printf("      本次导出：%s\n",
    exported_is_feasible ? "物理可行版（--feasible）" :
    "力矩拟合最优版（默认；**含非物理惯量**，仅供验证辨识效果）");

  // -----------------------------------------------------------------------
  // 7) 导出 URDF。
  // -----------------------------------------------------------------------
  const kdl_identification::UrdfExportResult exported = kdl_identification::exportUrdfWithParameters(
    urdf_in, prior_chain, beta_export, urdf_out);
  if (!exported.success) {
    std::printf("[7] 导出失败：%s\n", exported.message.c_str());
    return 1;
  }
  std::printf("\n[7] 已导出：%s\n", exported.output_file.c_str());
  std::printf("      %-7s %-12s %-28s %s\n", "link", "mass[kg]", "cog[m]", "replaced");
  for (const kdl_identification::UrdfLinkExport & link : exported.links) {
    char cog_text[96];
    std::snprintf(cog_text, sizeof(cog_text), "(%.5f, %.5f, %.5f)", link.cog.x(), link.cog.y(),
      link.cog.z());
    std::printf("      %-7s %-12.6f %-28s %s\n", link.link_name.c_str(), link.mass, cog_text,
      link.replaced_existing ? "yes" : "no(新增)");
  }

  // -----------------------------------------------------------------------
  // 8) 往返校验：重新解析导出的 URDF，确认写进去的参数没走样。
  // -----------------------------------------------------------------------
  const kdl_identification::UrdfRoundTripCheck round_trip =
    kdl_identification::verifyExportedUrdf(urdf_out, beta_export);
  if (!round_trip.success) {
    std::printf("[8] 往返校验失败：%s\n", round_trip.message.c_str());
    return 1;
  }
  std::printf(
    "\n[8] 往返校验（重新解析导出 URDF）：max|Δβ| = %.3e，max 相对误差 = %.3e → %s\n",
    round_trip.max_parameter_error, round_trip.max_relative_error,
    round_trip.max_parameter_error < kRoundTripTolerance ? "通过" : "未通过");

  // 确认真正写出去的那份参数是物理可行的（前面若走了交替投影，这里应当通过）。
  {
    const kdl_identification::PhysicalFeasibility feasibility =
      kdl_identification::checkPhysicalFeasibility(beta_export);
    std::printf("      导出参数的物理性检查：%s", feasibility.feasible ? "通过" : "**未通过**");
    if (feasibility.feasible) {
      std::printf("（最小质量 %.6f kg，质心惯量最小特征值 %.3e）", feasibility.min_mass,
        feasibility.min_inertia_eigenvalue);
    } else {
      std::printf("  ← %s", feasibility.message.c_str());
    }
    std::printf("\n");
  }

  // -----------------------------------------------------------------------
  // 9) 留出验证：换一条**不同的**激励轨迹，比较三个模型的力矩预测。
  //    这是"参数辨识有什么用"的核心证据。
  // -----------------------------------------------------------------------
  kdl_identification::ExcitationOptions holdout_options = excitation_options;
  holdout_options.random_seed = 20261111u;
  holdout_options.harmonics = 4;
  holdout_options.period = 3.0;
  const kdl_identification::ExcitationResult holdout_excitation =
    kdl_identification::optimizeFourierExcitation(prior_chain, limits, holdout_options);
  if (!holdout_excitation.success) {
    std::printf("[9] 留出轨迹优化失败：%s\n", holdout_excitation.message.c_str());
    return 1;
  }
  std::vector<kdl_identification::RegressorSample> holdout_samples;
  holdout_excitation.trajectory.samplePeriod(kHoldoutSamples, holdout_samples);

  // 真值力矩（留出轨迹上，真值模型说了算）。
  const Eigen::VectorXd tau_holdout = torqueOf(true_chain, holdout_samples);

  // 三个模型的预测：先验（原 URDF）、辨识（参数向量直接回代）、导出（重新解析的新 URDF）。
  const Eigen::VectorXd tau_prior_model = torqueOf(prior_chain, holdout_samples);
  KDL::Chain exported_chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_out, exported_chain)) {
    std::printf("[9] 无法从导出的 URDF 建链\n");
    return 1;
  }
  const Eigen::VectorXd tau_export_model = torqueOf(exported_chain, holdout_samples);
  // 另外用"回归矩阵 × 参数"回代一次：与上面那条解析导出的 URDF 路径互为印证。
  const kdl_identification::RegressorResult holdout_regressor = builder.build(holdout_samples);
  const Eigen::VectorXd tau_from_parameters =
    holdout_regressor.success ? (holdout_regressor.regressor * beta_export) : Eigen::VectorXd();

  const ErrorStats prior_error = errorStats(tau_prior_model, tau_holdout);
  const ErrorStats export_error = errorStats(tau_export_model, tau_holdout);
  const ErrorStats parameter_error = holdout_regressor.success ?
    errorStats(tau_from_parameters, tau_holdout) : ErrorStats();

  std::printf("\n[9] 留出验证（%zu 个新采样点 × %u 关节，激励轨迹与训练时不同）\n",
    holdout_samples.size(), n);
  std::printf("      %-34s %-14s %-14s\n", "模型", "残差 RMS", "残差 MAX");
  std::printf("      %-34s %-14.6e %-14.6e\n", "先验模型（原 URDF）", prior_error.rms, prior_error.max);
  std::printf("      %-34s %-14.6e %-14.6e\n", "辨识模型（W·β_补全）", parameter_error.rms,
    parameter_error.max);
  std::printf("      %-34s %-14.6e %-14.6e\n", "辨识模型（解析导出的 URDF）", export_error.rms,
    export_error.max);
  std::printf(
    "      改善倍数：RMS ×%.1f，MAX ×%.1f\n",
    export_error.rms > 0.0 ? prior_error.rms / export_error.rms : 0.0,
    export_error.max > 0.0 ? prior_error.max / export_error.max : 0.0);

  // -----------------------------------------------------------------------
  // 10) 教学要点：β_导出 ≠ β_真值，但 τ 一致 —— 所以"逐参数对比"是错的判据。
  // -----------------------------------------------------------------------
  const auto worst = worstParameterDifference(beta_export, beta_true);
  const double worst_relative_of_all = worst.second;
  std::printf(
    "\n[10] 注意：导出的参数与真值参数**并不相等**\n"
    "      最大单参数相对差异 = %.1f%%（%s）\n"
    "      但两者在留出轨迹上的力矩差异只有 %.3e N·m（RMS）。\n"
    "      原因是串联臂只有 %u 个基参数可辨识，其余方向对 τ 没有任何影响；\n"
    "      辨识成功与否的正确判据是**能否复现 τ**，不是逐参数是否对得上。\n",
    100.0 * worst_relative_of_all,
    kdl_identification::parameterLabel(prior_chain, worst.first).c_str(),
    export_error.rms, solution.rank);

  // -----------------------------------------------------------------------
  // 11) 结论：把"辨识有效"和"导出物是否物理"两件事分开说清楚。
  // -----------------------------------------------------------------------
  const bool round_trip_ok = round_trip.max_parameter_error < kRoundTripTolerance;
  // 用"至少好一倍"作为"辨识起作用"的门槛：否则微小的数值差异会被误判成改善
  // （--feasible 模式下导出物退化成先验，残差几乎不变，不能算"显著"）。
  const bool identification_effective =
    export_error.rms < 0.5 * prior_error.rms;
  std::printf(
    "\n结论：\n"
    "  · 导出管线：%s（往返 max|Δβ| = %.3e）\n"
    "  · 参数辨识的作用：%s（留出轨迹力矩 RMS %.3e → %.3e N·m，改善 ×%.1f）\n"
    "  · 导出物物理可行性：%s\n",
    round_trip_ok ? "正确" : "**异常**", round_trip.max_parameter_error,
    identification_effective ? "显著" : "**未体现**", prior_error.rms, export_error.rms,
    export_error.rms > 0.0 ? prior_error.rms / export_error.rms : 0.0,
    exported_is_feasible ? "通过" : "**未通过**（含非物理惯量，见上方说明）");

  if (!exported_is_feasible && identification_effective) {
    std::printf(
      "  说明：本次导出的是\"力矩拟合最优\"的那份参数，它精确复现 τ 但不满足\n"
      "        物理可行性（质心惯量需正定）。加 --feasible 可导出物理可行的版本，\n"
      "        但代价是力矩拟合退化回先验水平 —— 这是本实验暴露出的真实取舍：\n"
      "        \"τ 等价的仿射集\"与\"物理可行集\"（一个 LMI）在先验附近几乎不相交，\n"
      "        要同时满足需要在 LMI 约束下做凸优化（SDP），本模块未引入该依赖。\n");
  }

  // 判据：导出管线必须正确，且辨识效果必须体现。
  const bool ok = round_trip_ok && identification_effective;
  return ok ? 0 : 1;
}
