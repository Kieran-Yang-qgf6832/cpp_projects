// Copyright (c) 2026, kdl_identification authors.
// 教学用途：kdl_lsq.hpp 中声明的函数在这里落地。
//
// 整个求解按下面的顺序走，每一步都对应头文件里的一个"现实问题"：
//
//   W ──列归一化──► Ws ──QR(列主元)──► 基参数下标 idx
//                    │
//                    └──SVD判秩──► r = |idx|
//                    │
//                    └──取 idx 列──► Wr ──(可选)增广正则行──► SVD 求解 ──► x_r
//                                                                        │
//                          β[idx] = x_r ./ 列范数  ◄────────────────────┘
//
// 为什么不直接用正规方程 (WᵀW+λI)x = Wᵀτ：那会把条件数平方，
// 对"最不可辨识方向"的数值精度伤害很大。

#include "kdl_lsq.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

namespace kdl_identification
{
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_identification";

}  // namespace

IdentificationResult solveLeastSquares(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & torque,
  const LeastSquaresOptions & options)
{
  IdentificationResult result;
  result.total_parameters = static_cast<unsigned int>(regressor.cols());
  result.samples = static_cast<unsigned int>(regressor.rows());
  result.parameters = Eigen::VectorXd::Zero(regressor.cols());
  result.prediction = Eigen::VectorXd::Zero(regressor.rows());

  if (regressor.rows() == 0 || regressor.cols() == 0) {
    result.message = "回归矩阵为空（没有采样点或没有参数）";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (torque.size() != regressor.rows()) {
    result.message = "实测力矩长度 " + std::to_string(torque.size()) + " 与回归矩阵行数 " +
      std::to_string(regressor.rows()) + " 不一致";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  const unsigned int cols = static_cast<unsigned int>(regressor.cols());

  // ---- 1) 列归一化：消除 kg 与 kg·m² 之间的量纲差异 ----
  // 注意"恒零列"要用**相对**阈值判定：某些参数（如第一关节连杆的 m·cx）
  // 在解析上完全不影响 τ，数值上只会剩下 ~1e-16 的浮点残渣。若直接按范数
  // 归一化，这些残渣会被放大成单位列，混进基参数集、解出 1e12 量级的垃圾值。
  Eigen::VectorXd column_norm(cols);
  for (unsigned int j = 0; j < cols; ++j) {
    column_norm(j) = regressor.col(j).norm();
  }
  const double max_norm = (cols > 0) ? column_norm.maxCoeff() : 0.0;
  std::vector<bool> effective(cols, false);
  Eigen::MatrixXd scaled = regressor;
  for (unsigned int j = 0; j < cols; ++j) {
    effective[j] = (max_norm > 0.0) && (column_norm(j) > max_norm * options.column_tolerance);
    if (effective[j]) {
      scaled.col(j) /= column_norm(j);
    } else {
      scaled.col(j).setZero();  // 恒零列：该参数不被任何采样激励
    }
  }

  // ---- 2) 判秩：对归一化矩阵做 SVD ----
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled);
  const Eigen::VectorXd singular_values = svd.singularValues();
  const double max_singular = (singular_values.size() > 0) ? singular_values(0) : 0.0;
  if (max_singular <= 0.0) {
    result.message = "回归矩阵全零（激励不足），无法辨识任何参数";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  unsigned int rank = 0;
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > options.rank_tolerance * max_singular) {
      ++rank;
    }
  }
  if (rank == 0) {
    result.message = "回归矩阵数值秩为 0（激励不足），无法辨识任何参数";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  result.rank = rank;

  // ---- 3) 基参数集：带列主元的 QR 给出最独立的 rank 列 ----
  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(scaled);
  const auto permutation = qr.colsPermutation();
  result.base_parameter_indices.clear();
  result.base_parameter_indices.reserve(rank);
  for (unsigned int k = 0; k < cols && result.base_parameter_indices.size() < rank; ++k) {
    const unsigned int candidate = static_cast<unsigned int>(permutation.indices()(k));
    if (effective[candidate]) {
      result.base_parameter_indices.push_back(candidate);
    }
  }
  if (result.base_parameter_indices.size() < rank) {
    // 理论上不会发生（秩不会超过非零列数），留一个显式保护，避免静默出错解。
    result.message = "基参数集提取失败：独立列数少于数值秩";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  const unsigned int basis = static_cast<unsigned int>(result.base_parameter_indices.size());
  Eigen::MatrixXd reduced(regressor.rows(), basis);
  for (unsigned int j = 0; j < basis; ++j) {
    reduced.col(j) = scaled.col(result.base_parameter_indices[j]);
  }

  // ---- 4) 求解（可选 Tikhonov 正则化，用增广行实现，避免平方条件数） ----
  Eigen::MatrixXd lhs;
  Eigen::VectorXd rhs;
  if (options.regularization > 0.0) {
    lhs.resize(reduced.rows() + basis, basis);
    lhs.topRows(reduced.rows()) = reduced;
    lhs.bottomRows(basis) = std::sqrt(options.regularization) * Eigen::MatrixXd::Identity(basis, basis);
    rhs = Eigen::VectorXd::Zero(reduced.rows() + basis);
    rhs.head(reduced.rows()) = torque;
  } else {
    lhs = reduced;
    rhs = torque;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> solver(
    lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd x_reduced = solver.solve(rhs);
  if (!x_reduced.allFinite()) {
    result.message = "最小二乘求解得到非有限值（矩阵病态，可尝试增大正则化系数）";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  // ---- 5) 反归一化，填回完整参数向量 ----
  for (unsigned int j = 0; j < basis; ++j) {
    const unsigned int index = result.base_parameter_indices[j];
    result.parameters(index) = x_reduced(j) / column_norm(index);
  }

  // ---- 6) 预测、残差、条件数 ----
  result.prediction = regressor * result.parameters;
  const Eigen::VectorXd residual = torque - result.prediction;
  result.residual_rms = std::sqrt(residual.squaredNorm() / static_cast<double>(residual.size()));
  result.residual_max = residual.cwiseAbs().maxCoeff();

  if (options.compute_condition_number) {
    const Eigen::JacobiSVD<Eigen::MatrixXd> reduced_svd(reduced);
    const Eigen::VectorXd sv = reduced_svd.singularValues();
    if (sv.size() > 0 && sv(sv.size() - 1) > std::numeric_limits<double>::min()) {
      result.scaled_condition_number = sv(0) / sv(sv.size() - 1);
    }
  }

  result.success = true;
  result.message.clear();
  return result;
}

}  // namespace kdl_identification
