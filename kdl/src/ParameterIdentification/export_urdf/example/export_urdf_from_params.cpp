// Copyright (c) 2026, kdl_identification authors.
// 教学工具：把 kdl_identify_node 导出的 identification_params.csv 变成一份 URDF。
//
// 这是"闭环复用"的第一步：辨识结果目前只落在 CSV 里，要喂给控制器
// （kdl_effort_controller 的 urdf_file 参数）就必须先变回 URDF。
//
// 与 export_urdf_demo 的分工：
//   export_urdf_demo         —— 造"真值 ≠ 先验"的离线场景，验证**算法**对不对；
//   export_urdf_from_params  —— 吃**真实**闭环跑出来的参数 CSV，产出可用 URDF。
//
// 用法：
//   ros2 run kdl_tools export_urdf_from_params
//   ros2 run kdl_tools export_urdf_from_params --params /path/to/identification_params.csv
//   ros2 run kdl_tools export_urdf_from_params --out /tmp/my.urdf
//
// 默认路径（编译期写死）：参数 CSV 取 draw 目录、URDF 取 model 目录、输出落本模块。
//
// ===========================================================================
// 为什么这里**必须**开 enforce_physical_feasibility
// ===========================================================================
// 最小二乘只保证 W·β = τ，不保证 β 物理可行；而"喂给控制器"要求 URDF 里的
// 惯量必须是真刚体。所以本工具默认开启可行性投影，并在失败时明确报错 ——
// 宁可拒绝产出一份会污染下游的 URDF，也不要静默写出一份负惯量的文件。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

#include "kdl_fourier.hpp"  // FourierLimits（仅用于取限位造随机采样点）
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

#ifndef KDL_IDENTIFY_DRAW_DIR
#define KDL_IDENTIFY_DRAW_DIR "."
#endif

using kdl_identification::kParamsPerLink;

/// 零空间补全用的随机采样点数。
constexpr unsigned int kCompletionSamples = 600;

/// 加速度上限 = 速度上限 × 该倍数（URDF 里没有加速度限位）。
constexpr double kAccelerationOverVelocity = 5.0;

/// 判定"导出正确"的参数往返阈值。
constexpr double kRoundTripTolerance = 1e-9;

/**
 * @brief 读 kdl_identify_node 导出的参数 CSV，重建完整参数向量。
 * @param path       [in]  CSV 路径（表头 index,label,value）。
 * @param parameters [in]  期望长度（= 10 × 段数）。
 * @param values     [out] 重建出的参数向量；非基参数填 0（与最小二乘解一致）。
 * @param friction   [out] 顺带读出的摩擦项（下标 → 值），仅用于打印。
 * @param message    [out] 失败原因。
 * @return true 表示成功。
 *
 * @note CSV 里的 index 是**增强矩阵**的列号：0..10·段数−1 是惯性，
 *       之后每关节两列（粘性、库伦）。惯性之外的下标这里直接忽略。
 */
bool loadParametersCsv(
  const std::string & path, unsigned int expected_size, Eigen::VectorXd & values,
  std::vector<std::pair<unsigned int, double>> & friction, std::string & message)
{
  std::FILE * file = std::fopen(path.c_str(), "r");
  if (file == nullptr) {
    message = "无法打开参数 CSV：" + path;
    return false;
  }

  values = Eigen::VectorXd::Zero(expected_size);
  friction.clear();

  char line[1024];
  bool header_skipped = false;
  unsigned int rows = 0;
  while (std::fgets(line, sizeof(line), file) != nullptr) {
    if (!header_skipped) {
      header_skipped = true;
      continue;  // 跳过表头
    }
    unsigned int index = 0;
    double value = 0.0;
    // 表头之后每行是 "index,label,value"；label 里没有逗号。
    if (std::sscanf(line, "%u,%*[^,],%lf", &index, &value) != 2) {
      continue;  // 空行/注释行
    }
    ++rows;
    if (index < expected_size) {
      values(index) = value;
    } else {
      friction.emplace_back(index, value);
    }
  }
  std::fclose(file);

  if (rows == 0) {
    message = "参数 CSV 里没有解析到任何数据行：" + path;
    return false;
  }
  message.clear();
  return true;
}

/**
 * @brief 从 URDF 读关节限位（只为在限位内造随机采样点）。
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
      continue;
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
  message.clear();
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string params_csv = std::string(KDL_IDENTIFY_DRAW_DIR) + "/identification_params.csv";
  std::string urdf_in = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  std::string urdf_out = std::string(KDL_EXPORT_URDF_DIR) + "/robotic_arm_identified.urdf";

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--params") == 0 && i + 1 < argc) {
      params_csv = argv[++i];
    } else if (std::strcmp(argv[i], "--urdf") == 0 && i + 1 < argc) {
      urdf_in = argv[++i];
    } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      urdf_out = argv[++i];
    }
  }

  std::printf("=== 辨识参数 → URDF（闭环复用第一步）===\n");
  std::printf("参数 CSV：%s\n", params_csv.c_str());
  std::printf("模板 URDF：%s\n", urdf_in.c_str());
  std::printf("输出 URDF：%s\n\n", urdf_out.c_str());

  std::string message;

  // ---- 1) 先验模型（模板 URDF 的原值）----
  KDL::Chain prior_chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_in, prior_chain)) {
    std::printf("建链失败：%s\n", urdf_in.c_str());
    return 1;
  }
  const unsigned int segments = prior_chain.getNrOfSegments();
  const unsigned int inertia_columns = kParamsPerLink * segments;

  Eigen::VectorXd beta_prior;
  if (!kdl_identification::extractParameters(prior_chain, beta_prior, message)) {
    std::printf("读取先验参数失败：%s\n", message.c_str());
    return 1;
  }

  // ---- 2) 读辨识结果 ----
  Eigen::VectorXd beta_identified;
  std::vector<std::pair<unsigned int, double>> friction;
  if (!loadParametersCsv(params_csv, inertia_columns, beta_identified, friction, message)) {
    std::printf("%s\n", message.c_str());
    return 1;
  }
  const unsigned int base_count =
    static_cast<unsigned int>((beta_identified.array() != 0.0).count());
  std::printf("[1] 读入 %u 个惯性参数列，其中非零（= 基参数）%u 个\n", inertia_columns,
    base_count);
  if (!friction.empty()) {
    std::printf("     顺带读到的摩擦项（不属于 URDF，仅打印）：\n");
    for (const auto & item : friction) {
      const unsigned int offset = item.first - inertia_columns;
      const unsigned int joint = offset % prior_chain.getNrOfJoints();
      const char * kind = (offset < prior_chain.getNrOfJoints()) ? "粘性" : "库伦";
      std::printf("       joint%u %s = %+.4f\n", joint + 1, kind, item.second);
    }
  }

  // ---- 3) 零空间补全：把不可辨识分量按先验补上，并保证物理可行 ----
  kdl_identification::FourierLimits limits;
  if (!readJointLimits(urdf_in, prior_chain, limits, message)) {
    std::printf("读取关节限位失败：%s\n", message.c_str());
    return 1;
  }

  const unsigned int n = prior_chain.getNrOfJoints();
  std::mt19937 generator(20261008u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::vector<kdl_identification::RegressorSample> samples;
  samples.reserve(kCompletionSamples);
  for (unsigned int i = 0; i < kCompletionSamples; ++i) {
    kdl_identification::RegressorSample sample;
    sample.q = KDL::JntArray(n);
    sample.qdot = KDL::JntArray(n);
    sample.qddot = KDL::JntArray(n);
    for (unsigned int j = 0; j < n; ++j) {
      const double middle = 0.5 * (limits.q_min(j) + limits.q_max(j));
      const double half = 0.45 * (limits.q_max(j) - limits.q_min(j));
      sample.q(j) = middle + half * unit(generator);
      sample.qdot(j) = 0.5 * limits.qdot_max(j) * unit(generator);
      sample.qddot(j) = 0.5 * limits.qddot_max(j) * unit(generator);
    }
    samples.push_back(std::move(sample));
  }

  kdl_identification::RegressorBuilder builder(prior_chain);
  if (!builder.valid()) {
    std::printf("回归矩阵构造器不可用：%s\n", builder.error().c_str());
    return 1;
  }
  const kdl_identification::RegressorResult regressor = builder.build(samples);
  if (!regressor.success) {
    std::printf("构建回归矩阵失败：%s\n", regressor.message.c_str());
    return 1;
  }

  kdl_identification::CompletionOptions options;
  options.enforce_physical_feasibility = true;  // 喂控制器必须物理可行，见文件头
  const kdl_identification::CompletionResult completion = kdl_identification::completeWithPrior(
    regressor.regressor, beta_identified, beta_prior, options);
  if (!completion.success) {
    std::printf("[2] 零空间补全失败：%s\n", completion.message.c_str());
    return 1;
  }
  std::printf(
    "[2] 零空间补全：秩 = %u，零空间维数 = %u\n"
    "      与先验的距离 %.3e（补全前 %.3e），退让比例 t = %.4f，力矩偏差 %.3e\n",
    completion.rank, completion.null_space_dimension, completion.correction_norm,
    completion.prior_gap_norm, completion.feasibility_fraction, completion.torque_residual_norm);

  const kdl_identification::PhysicalFeasibility feasibility =
    kdl_identification::checkPhysicalFeasibility(completion.parameters);
  std::printf("[3] 物理可行性：%s", feasibility.feasible ? "通过" : "**未通过**");
  if (feasibility.feasible) {
    std::printf("（最小质量 %.6f kg，质心惯量最小特征值 %.3e）\n", feasibility.min_mass,
      feasibility.min_inertia_eigenvalue);
  } else {
    std::printf("  ← %s\n", feasibility.message.c_str());
    std::printf("      拒绝导出：这份 URDF 会污染下游（控制器/仿真会拿到非物理惯量）\n");
    return 1;
  }

  // ---- 4) 导出 + 往返校验 ----
  const kdl_identification::UrdfExportResult exported = kdl_identification::exportUrdfWithParameters(
    urdf_in, prior_chain, completion.parameters, urdf_out);
  if (!exported.success) {
    std::printf("[4] 导出失败：%s\n", exported.message.c_str());
    return 1;
  }
  std::printf("\n[4] 已导出：%s\n", exported.output_file.c_str());
  std::printf("      %-7s %-12s %-30s %s\n", "link", "mass[kg]", "cog[m]", "相对先验");
  for (unsigned int s = 0; s < segments; ++s) {
    const kdl_identification::UrdfLinkExport & link = exported.links[s];
    const double prior_mass = beta_prior[kParamsPerLink * s];
    // 注意：这里不能用 `cond ? expr : Vector3d::Zero()` —— 三元运算符要求两个分支
    // 类型一致，而 Eigen 的除法结果是表达式模板，与 ConstantReturnType 不同，
    // 会直接编译失败。先算再赋值。
    Eigen::Vector3d prior_cog = Eigen::Vector3d::Zero();
    if (prior_mass > 0.0) {
      prior_cog = Eigen::Vector3d(beta_prior[kParamsPerLink * s + 1],
        beta_prior[kParamsPerLink * s + 2], beta_prior[kParamsPerLink * s + 3]) / prior_mass;
    }
    char text[128];
    std::snprintf(text, sizeof(text), "(%.5f, %.5f, %.5f)", link.cog.x(), link.cog.y(), link.cog.z());
    std::printf("      %-7s %-12.6f %-30s 质量 %+.1f%%  质心 %+.4f m\n", link.link_name.c_str(),
      link.mass, text, 100.0 * (link.mass - prior_mass) / prior_mass,
      (link.cog - prior_cog).norm());
  }

  const kdl_identification::UrdfRoundTripCheck round_trip =
    kdl_identification::verifyExportedUrdf(urdf_out, completion.parameters);
  if (!round_trip.success) {
    std::printf("[5] 往返校验失败：%s\n", round_trip.message.c_str());
    return 1;
  }
  std::printf("\n[5] 往返校验：max|Δβ| = %.3e → %s\n", round_trip.max_parameter_error,
    round_trip.max_parameter_error < kRoundTripTolerance ? "通过" : "**未通过**");

  const bool ok = round_trip.max_parameter_error < kRoundTripTolerance && feasibility.feasible;
  std::printf(
    "\n结论：%s\n"
    "  下一步（闭环复用第二步）把控制器的 urdf_file 指向这份文件：\n"
    "    kdl_effort_controller.ros__parameters.urdf_file = %s\n",
    ok ? "已产出一份物理可行、往返无损的 URDF" : "未通过，请检查上方数值", urdf_out.c_str());
  return ok ? 0 : 1;
}
