// Copyright (c) 2026, kdl_identification authors.
// 教学示例：为参数辨识生成一条傅里叶激励轨迹，并对比"优化前/后"的可观测性。
//
// 流程：
//   1) 从 URDF 读关节限位（位置上下限、速度上限），加速度上限取速度上限的 5 倍；
//   2) 造一条朴素基线（每个关节只用一次谐波、等幅、缩放到贴住约束）；
//   3) 用 optimizeFourierExcitation() 优化出一条更好的激励轨迹；
//   4) 打印两者的 rank / σ_min⁺ / 条件数，并把最优轨迹的一个周期导出成 CSV。
//
// 运行：
//   colcon build --paths .
//   source install/setup.bash
//   ros2 run kdl_tools fourier_demo

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_fourier.hpp"
#include "kdl_regressor.hpp"
#include "kdl_tools.hpp"

namespace
{

#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

/// 加速度上限没有写在 URDF 里，用速度上限的这个倍数作为工程近似。
constexpr double kAccelerationOverVelocity = 5.0;

/// CSV 采样行数（含两端）。
constexpr int kCsvRows = 401;

/**
 * @brief 从 urdf::Model 读关节限位，按链的关节顺序填入。
 * @return true 表示全部读到。
 */
bool readJointLimits(
  const urdf::Model & model, const KDL::Chain & chain, KDL::JntArray & q_min,
  KDL::JntArray & q_max, KDL::JntArray & qdot_max)
{
  const unsigned int n = chain.getNrOfJoints();
  q_min.resize(n);
  q_max.resize(n);
  qdot_max.resize(n);

  unsigned int index = 0;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // 固定段不占自由度
    }
    const auto it = model.joints_.find(joint.getName());
    if (it == model.joints_.end() || !it->second->limits) {
      return false;
    }
    q_min(index) = it->second->limits->lower;
    q_max(index) = it->second->limits->upper;
    qdot_max(index) = it->second->limits->velocity;
    ++index;
  }
  return index == n;
}

/**
 * @brief 把一个周期的轨迹导出成 CSV（t, q0.., qdot0.., qddot0..）。
 */
bool writeCsv(const kdl_identification::FourierTrajectory & trajectory, const std::string & path)
{
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }

  const unsigned int n = trajectory.joints();
  csv << "t";
  for (unsigned int j = 0; j < n; ++j) {
    csv << ",q" << j;
  }
  for (unsigned int j = 0; j < n; ++j) {
    csv << ",qdot" << j;
  }
  for (unsigned int j = 0; j < n; ++j) {
    csv << ",qddot" << j;
  }
  csv << "\n";

  for (int k = 0; k < kCsvRows; ++k) {
    const double t = trajectory.period() * k / (kCsvRows - 1);
    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qddot;
    trajectory.sample(t, q, qdot, qddot);
    csv << t;
    for (unsigned int j = 0; j < n; ++j) {
      csv << "," << q(j);
    }
    for (unsigned int j = 0; j < n; ++j) {
      csv << "," << qdot(j);
    }
    for (unsigned int j = 0; j < n; ++j) {
      csv << "," << qddot(j);
    }
    csv << "\n";
  }
  return true;
}

/// 打印一行可观测性对比。
void printObservability(const char * tag, const kdl_identification::ObservabilityReport & report)
{
  std::printf(
    "  %-14s rank = %u / %u   σ_min⁺ = %.4e   cond = %.4e\n", tag, report.rank,
    report.total_parameters, report.min_singular_value, report.condition_number);
}

}  // namespace

int main()
{
  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";

  KDL::Chain chain;
  urdf::Model model;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain) ||
    !kdl_tools::loadUrdfModel(urdf_file, model))
  {
    std::printf("建链或解析 URDF 失败：%s\n", urdf_file.c_str());
    return 1;
  }

  const unsigned int joints = chain.getNrOfJoints();
  KDL::JntArray q_min;
  KDL::JntArray q_max;
  KDL::JntArray qdot_max;
  if (!readJointLimits(model, chain, q_min, q_max, qdot_max)) {
    std::printf("读取关节限位失败\n");
    return 1;
  }
  KDL::JntArray qddot_max(joints);
  for (unsigned int j = 0; j < joints; ++j) {
    qddot_max(j) = kAccelerationOverVelocity * qdot_max(j);
  }

  kdl_identification::FourierLimits limits;
  limits.q_min = q_min;
  limits.q_max = q_max;
  limits.qdot_max = qdot_max;
  limits.qddot_max = qddot_max;

  std::printf("=== 傅里叶激励轨迹 ===\n");
  std::printf("链：%u 自由度 / %u 段\n", joints, chain.getNrOfSegments());
  std::printf("约束（来自 URDF；加速度上限 = 速度上限 × %.0f）：\n", kAccelerationOverVelocity);
  for (unsigned int j = 0; j < joints; ++j) {
    std::printf(
      "  joint%u: q ∈ [%+.3f, %+.3f] rad, |q̇| ≤ %.3f rad/s, |q̈| ≤ %.3f rad/s²\n", j + 1,
      q_min(j), q_max(j), qdot_max(j), qddot_max(j));
  }

  kdl_identification::ExcitationOptions options;
  options.harmonics = 5;
  options.period = 4.0;
  options.samples_per_period = 50;
  options.restarts = 15;
  options.refine_iterations = 8;
  options.max_evaluations = 150;

  std::printf(
    "\n选项：H = %u, T = %.3f s, 评估采样 = %u/周期, 重启 = %u, 局部步 = %u\n", options.harmonics,
    options.period, options.samples_per_period, options.restarts, options.refine_iterations);

  // 回归矩阵构造器只造一次，供基线与优化共用。
  const kdl_identification::RegressorBuilder builder(chain, options.gravity);
  if (!builder.valid()) {
    std::printf("回归矩阵构造器不可用：%s\n", builder.error().c_str());
    return 1;
  }

  // ---- 基线：每个关节只用一次谐波、等幅，缩放到贴住约束 ----
  KDL::JntArray center(joints);
  for (unsigned int j = 0; j < joints; ++j) {
    center(j) = 0.5 * (q_min(j) + q_max(j));
  }
  Eigen::MatrixXd baseline_a = Eigen::MatrixXd::Zero(joints, options.harmonics);
  Eigen::MatrixXd baseline_b = Eigen::MatrixXd::Zero(joints, options.harmonics);
  baseline_a.col(0).setOnes();
  const kdl_identification::FourierTrajectory baseline_unit(
    options.harmonics, options.period, center, baseline_a, baseline_b);
  const kdl_identification::FourierScaling baseline_scaling = kdl_identification::scaleToFitLimits(
    baseline_unit, limits, options.samples_per_period);
  const kdl_identification::FourierTrajectory baseline =
    baseline_unit.scaled(baseline_scaling.scale);
  const kdl_identification::ObservabilityReport baseline_report =
    kdl_identification::evaluateObservability(builder, baseline, 200);

  // ---- 自检：sample() 的解析 q̇/q̈ 应当与中心差分一致 ----
  {
    const double h = 1e-6;
    double max_velocity_error = 0.0;
    double max_acceleration_error = 0.0;
    for (int k = 0; k < 37; ++k) {
      const double t = baseline_unit.period() * k / 36.0;
      KDL::JntArray q;
      KDL::JntArray qdot;
      KDL::JntArray qddot;
      KDL::JntArray q_plus;
      KDL::JntArray qdot_plus;
      KDL::JntArray qddot_plus;
      KDL::JntArray q_minus;
      KDL::JntArray qdot_minus;
      KDL::JntArray qddot_minus;
      baseline_unit.sample(t, q, qdot, qddot);
      baseline_unit.sample(t + h, q_plus, qdot_plus, qddot_plus);
      baseline_unit.sample(t - h, q_minus, qdot_minus, qddot_minus);
      for (unsigned int j = 0; j < joints; ++j) {
        max_velocity_error = std::max(
          max_velocity_error, std::abs(qdot(j) - (q_plus(j) - q_minus(j)) / (2.0 * h)));
        max_acceleration_error = std::max(
          max_acceleration_error,
          std::abs(qddot(j) - (qdot_plus(j) - qdot_minus(j)) / (2.0 * h)));
      }
    }
    std::printf(
      "\n[自检] 解析导数 vs 中心差分：max|q̇| 误差 = %.3e，max|q̈| 误差 = %.3e\n",
      max_velocity_error, max_acceleration_error);
  }

  // ---- 优化 ----
  const kdl_identification::ExcitationResult best =
    kdl_identification::optimizeFourierExcitation(chain, limits, options);
  if (!best.success) {
    std::printf("\n优化失败：%s\n", best.message.c_str());
    return 1;
  }

  std::printf("\n目标函数评估次数：%u\n", best.evaluations);
  std::printf("可观测性对比：\n");
  printObservability("基线", baseline_report);
  printObservability("优化后", best.observability);

  std::printf("\n最优轨迹：缩放系数 λ = %.4f\n", best.scale);
  std::printf(
    "约束校验（比值，1 为上限）：位置 %.4f / 速度 %.4f / 加速度 %.4f（最紧的关节 #%u）\n",
    best.validation.position_ratio, best.validation.velocity_ratio,
    best.validation.acceleration_ratio, best.validation.worst_joint + 1);

  // 峰值（物理量）。
  double qdot_peak = 0.0;
  double qddot_peak = 0.0;
  std::vector<kdl_identification::RegressorSample> samples;
  best.trajectory.samplePeriod(400, samples);
  for (const auto & sample : samples) {
    for (unsigned int j = 0; j < joints; ++j) {
      qdot_peak = std::max(qdot_peak, std::abs(sample.qdot(j)));
      qddot_peak = std::max(qddot_peak, std::abs(sample.qddot(j)));
    }
  }
  std::printf("峰值：|q̇| = %.4f rad/s, |q̈| = %.4f rad/s²\n", qdot_peak, qddot_peak);

  // ---- 导出 CSV ----
  const std::string csv_path = std::filesystem::absolute("fourier_excitation.csv").string();
  if (writeCsv(best.trajectory, csv_path)) {
    std::printf(
      "\n已写入：%s\n  列含义：t, q0..q%u, qdot0..qdot%u, qddot0..qddot%u（%d 行，一个周期）\n",
      csv_path.c_str(), joints - 1, joints - 1, joints - 1, kCsvRows);
  } else {
    std::printf("CSV 写入失败：%s\n", csv_path.c_str());
  }

  std::printf(
    "\n结论：优化把最弱方向的信号强度从 %.4e 提到 %.4e（基参数个数 %u → %u）\n",
    baseline_report.min_singular_value, best.observability.min_singular_value,
    baseline_report.rank, best.observability.rank);
  return 0;
}
