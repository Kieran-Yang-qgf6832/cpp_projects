// Copyright (c) 2026, kdl_identification authors.
// 教学示例：回归矩阵 + 最小二乘的**离线自检**。
//
// 不需要 MuJoCo、不需要 ROS 话题：直接用 URDF 里的真值参数当"被测系统"，
// 随机造一批运动状态，用逆动力学算出"理论力矩"，再走一遍完整的辨识流程。
// 这相当于把辨识算法放在"已知答案"的环境里过一遍。
//
// 三件事依次验证：
//   1) 参数 <-> 链 的往返换算是否无损（平行轴定理写对了没有）；
//   2) 回归矩阵是否自洽：Y·β_真值 == 逆动力学算出来的 τ（应到机器精度）；
//   3) 最小二乘能否复现力矩，且在**留出的新采样点**上预测同样准确。
//
// ⚠️ 一个必须建立的概念：**单个物理参数不可辨识**。
//    串联臂的参数向量 β 里，只有少数独立组合（基参数，base parameters）
//    真正影响 τ。所以辨识出来的"基参数值"并不等于 URDF 里那一行行质量/质心/
//    惯量——把它们逐项对比是**错的**。辨识成功的标准是：辨识结果能在任意
//    运动状态下复现真实的 τ（本示例用留出集验证这一点）。
//
// 运行：
//   colcon build --paths .
//   source install/setup.bash
//   ros2 run kdl_tools regressor_lsq_demo

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "kdl_idynamics.hpp"
#include "kdl_lsq.hpp"
#include "kdl_regressor.hpp"
#include "kdl_tools.hpp"

namespace
{

#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

/// 辨识用的采样点数（训练集）。
constexpr int kNumTrainSamples = 300;

/// 验证用的新采样点数（留出集，不参与求解）。
constexpr int kNumTestSamples = 100;

/**
 * @brief 随机造一批采样点，并用真值链算出力矩。
 * @param chain      [in]  真值链。
 * @param count      [in]  采样点数。
 * @param generator  [in]  随机数发生器。
 * @param samples    [out] 采样点。
 * @param torque     [out] 堆叠的真实力矩，长度 = count × 关节数。
 */
void makeDataset(
  const KDL::Chain & chain, int count, std::mt19937 & generator,
  std::vector<kdl_identification::RegressorSample> & samples, Eigen::VectorXd & torque)
{
  const unsigned int joints = chain.getNrOfJoints();
  std::uniform_real_distribution<double> q_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> qd_dist(-0.8, 0.8);
  std::uniform_real_distribution<double> qdd_dist(-1.0, 1.0);

  samples.clear();
  samples.reserve(count);
  torque.resize(static_cast<Eigen::Index>(count) * joints);

  for (int i = 0; i < count; ++i) {
    kdl_identification::RegressorSample sample;
    sample.q = KDL::JntArray(joints);
    sample.qdot = KDL::JntArray(joints);
    sample.qddot = KDL::JntArray(joints);
    for (unsigned int j = 0; j < joints; ++j) {
      sample.q(j) = q_dist(generator);
      sample.qdot(j) = qd_dist(generator);
      sample.qddot(j) = qdd_dist(generator);
    }

    const kdl_dynamics::IdResult id =
      kdl_dynamics::inverseDynamics(chain, sample.q, sample.qdot, sample.qddot);
    for (unsigned int j = 0; j < joints; ++j) {
      torque(static_cast<Eigen::Index>(i) * joints + j) = id.torque(j);
    }
    samples.push_back(sample);
  }
}

}  // namespace

int main()
{
  // -----------------------------------------------------------------------
  // 0) 建模：与其它示例一样，从源码树的 URDF 建链（base_link -> link6）。
  // -----------------------------------------------------------------------
  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    std::printf("建链失败：%s\n", urdf_file.c_str());
    return 1;
  }

  const unsigned int joints = chain.getNrOfJoints();
  const unsigned int segments = chain.getNrOfSegments();
  const unsigned int params = kdl_identification::parameterCount(chain);
  std::printf("链：%u 自由度 / %u 段 / %u 个待辨识参数\n", joints, segments, params);
  std::printf("URDF：%s\n\n", urdf_file.c_str());

  std::string message;

  // -----------------------------------------------------------------------
  // 1) 真值参数，以及"参数 -> 链 -> 参数"的往返自检。
  // -----------------------------------------------------------------------
  Eigen::VectorXd beta_true;
  if (!kdl_identification::extractParameters(chain, beta_true, message)) {
    std::printf("读取真值参数失败：%s\n", message.c_str());
    return 1;
  }

  KDL::Chain rebuilt_chain;
  if (!kdl_identification::applyParameters(chain, beta_true, rebuilt_chain, message)) {
    std::printf("按参数重建链失败：%s\n", message.c_str());
    return 1;
  }
  Eigen::VectorXd beta_roundtrip;
  kdl_identification::extractParameters(rebuilt_chain, beta_roundtrip, message);
  const double roundtrip_error = (beta_roundtrip - beta_true).cwiseAbs().maxCoeff();
  std::printf("[1] 参数往返最大误差          = %.3e\n", roundtrip_error);

  // -----------------------------------------------------------------------
  // 2) 训练集：随机运动 -> 真值链的逆动力学给出"实测"力矩。
  // -----------------------------------------------------------------------
  std::mt19937 generator(20261005u);
  std::vector<kdl_identification::RegressorSample> train_samples;
  Eigen::VectorXd train_torque;
  makeDataset(chain, kNumTrainSamples, generator, train_samples, train_torque);
  std::printf("[2] 训练采样点数              = %d\n", kNumTrainSamples);

  // -----------------------------------------------------------------------
  // 3) 构建堆叠回归矩阵，并检查自洽性 Y·β_真值 == τ。
  // -----------------------------------------------------------------------
  const kdl_identification::RegressorResult train_regressor =
    kdl_identification::buildRegressor(chain, train_samples);
  if (!train_regressor.success) {
    std::printf("构建回归矩阵失败：%s\n", train_regressor.message.c_str());
    return 1;
  }
  const double regressor_consistency =
    ((train_regressor.regressor * beta_true) - train_torque).cwiseAbs().maxCoeff();
  std::printf(
    "[3] 回归矩阵自洽性 max|Y·β−τ|  = %.3e （应接近 0）\n", regressor_consistency);

  // -----------------------------------------------------------------------
  // 4) 最小二乘辨识。
  // -----------------------------------------------------------------------
  const kdl_identification::IdentificationResult solution =
    kdl_identification::solveLeastSquares(train_regressor.regressor, train_torque);
  if (!solution.success) {
    std::printf("辨识失败：%s\n", solution.message.c_str());
    return 1;
  }
  std::printf("[4] 数值秩（可辨识基参数）    = %u / %u\n", solution.rank, solution.total_parameters);
  std::printf("    训练集力矩残差 RMS        = %.3e N·m\n", solution.residual_rms);
  std::printf("    训练集力矩残差 MAX        = %.3e N·m\n", solution.residual_max);
  std::printf("    基参数矩阵条件数          = %.3e\n", solution.scaled_condition_number);

  // -----------------------------------------------------------------------
  // 5) 留出验证：用没参与求解的新采样点检验"辨识出的模型能否复现 τ"。
  //    这才是辨识成功与否的判据（不是逐参数对比）。
  // -----------------------------------------------------------------------
  std::vector<kdl_identification::RegressorSample> test_samples;
  Eigen::VectorXd test_torque;
  makeDataset(chain, kNumTestSamples, generator, test_samples, test_torque);
  const kdl_identification::RegressorResult test_regressor =
    kdl_identification::buildRegressor(chain, test_samples);
  if (!test_regressor.success) {
    std::printf("构建验证集回归矩阵失败：%s\n", test_regressor.message.c_str());
    return 1;
  }
  const double test_error =
    ((test_regressor.regressor * solution.parameters) - test_torque).cwiseAbs().maxCoeff();
  std::printf(
    "\n[5] 留出集力矩预测 max 误差   = %.3e N·m （%d 个新采样点）\n", test_error,
    kNumTestSamples);

  // -----------------------------------------------------------------------
  // 6) 概念说明：为什么单个参数对不上。
  // -----------------------------------------------------------------------
  std::printf(
    "\n[6] 说明：%u 个参数里只有 %u 个基参数可辨识。\n"
    "    串联臂的单个质量/质心/惯量并不独立影响关节力矩，只有它们的某些组合\n"
    "    （基参数）能被唯一确定。因此下面这些值**不等于** URDF 里的对应项：\n",
    solution.total_parameters, solution.rank);

  const unsigned int shown = std::min<unsigned int>(5, solution.base_parameter_indices.size());
  for (unsigned int i = 0; i < shown; ++i) {
    const unsigned int index = solution.base_parameter_indices[i];
    std::printf(
      "      %-16s 基参数值 = %+.6e   （URDF 对应项 = %+.6e）\n",
      kdl_identification::parameterLabel(chain, index).c_str(), solution.parameters(index),
      beta_true(index));
  }

  const bool ok = roundtrip_error < 1e-9 && regressor_consistency < 1e-9 &&
    solution.residual_max < 1e-9 && test_error < 1e-9;
  std::printf(
    "\n结论：%s\n", ok ? "回归矩阵与最小二乘自检通过（辨识模型能在新采样点上复现真实力矩）" :
    "自检未通过，请检查上方数值");
  return ok ? 0 : 1;
}
