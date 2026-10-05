// Copyright (c) 2026, kdl_identification authors.
// 教学用途：惯性参数辨识的求解部分（最小二乘 + 基参数提取）。
//
// ===========================================================================
// 一、要解的问题
// ===========================================================================
// 有了回归矩阵（kdl_regressor.hpp）和实测力矩，辨识就是解一个最小二乘：
//
//     W β = τ_meas        W 的行数 = 采样点数 × 关节数，列数 = 10 × 段数
//
// 但**不能直接** (WᵀW)⁻¹ Wᵀ τ 了事，因为有两个现实问题：
//
//   1) 秩亏：串联机械臂必然有一批惯性参数（或其组合）不影响任何关节力矩。
//      例如绕同一转轴的连杆参数、固定段参数——它们对应的列线性相关或恒为 0。
//      这时 WᵀW 奇异，直接求逆会得到荒谬的解。
//   2) 量纲差异：质量的量级是 kg，转动惯量是 kg·m²，数值上差好几个数量级，
//      导致 WᵀW 的条件数被人为放大。
//
// ===========================================================================
// 二、对策
// ===========================================================================
//   列归一化：把每列除以它的范数，消除量纲差异（WᵀW 的条件数只由
//             "方向相关性"决定）。
//   判秩：对归一化后的矩阵做奇异值分解，按最大奇异值的相对阈值判秩 r。
//   基参数集：用带列主元的 QR 找出"最独立"的 r 列，它们就是**基参数**
//             （base parameters）——真正可辨识的那些（或它们的组合）。
//             其余 r+1 列起的参数置 0（不可辨识，改动它们不影响 τ）。
//   求解：只在基参数列上解超定/病态系统。用 SVD（截断）保证数值稳定，
//         可选 Tikhonov 正则化（把问题变成"带阻尼最小二乘"）。
//
// ===========================================================================
// 三、输出怎么读
// ===========================================================================
// IdentificationResult::parameters 是完整长度的 β（不可辨识的分量填 0），
// base_parameter_indices 给出哪些下标是"独立、真正辨识出来"的。
// 判定好坏看两个量：
//   residual_rms / residual_max —— 预测 τ 与实测 τ 的残差（越小越好）；
//   scaled_condition_number     —— 基参数矩阵的条件数（越大越不可信）。

#ifndef KDL_PARAMETER_IDENTIFICATION__KDL_LSQ_HPP_
#define KDL_PARAMETER_IDENTIFICATION__KDL_LSQ_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

namespace kdl_identification
{

/**
 * @brief 最小二乘求解的可调参数。
 */
struct LeastSquaresOptions
{
  /// 判秩的相对阈值：奇异值 < rank_tolerance × 最大奇异值 视为 0。
  /// 回归矩阵是解析构造的（几乎无噪声），默认给得很小；实测数据可放宽到 1e-6。
  double rank_tolerance = 1e-9;

  /// 列范数的相对阈值：某列范数 < column_tolerance × 最大列范数 视为"恒零列"
  /// （该参数在任何采样点上都不影响 τ，属不可辨识），直接置零、不参与归一化。
  /// 这一条很重要：若不做，浮点噪声列（~1e-16）会被归一化放大成单位列，
  /// 混进基参数集并解出 1e12 量级的"垃圾值"。
  double column_tolerance = 1e-10;

  /// Tikhonov 正则化系数 λ ≥ 0（作用于归一化后的基参数矩阵）。
  /// 0 表示不正则化；数据噪声大 / 条件数极差时可取小正数换来更稳的解。
  double regularization = 0.0;

  /// 是否计算基参数矩阵的条件数（需要一次额外 SVD，教学场景开销可忽略）。
  bool compute_condition_number = true;
};

/**
 * @brief 参数辨识的求解结果。
 */
struct IdentificationResult
{
  /// 完整长度的参数向量 β（不可辨识的分量填 0）。
  Eigen::VectorXd parameters;

  /// 预测力矩 W·β，长度与实测 τ 一致，便于做残差/曲线对比。
  Eigen::VectorXd prediction;

  /// 被判定为可辨识的列下标（基参数集），升序之外的顺序由 QR 主元给出。
  std::vector<unsigned int> base_parameter_indices;

  unsigned int rank = 0;              ///< 数值秩 = 基参数个数
  unsigned int total_parameters = 0;  ///< W 的列数 = 10 × 段数
  unsigned int samples = 0;           ///< W 的行数 = 采样点数 × 关节数

  double residual_rms = 0.0;  ///< 残差的均方根，单位与 τ 相同（N·m）
  double residual_max = 0.0;  ///< 残差的最大绝对值

  /// 归一化后基参数矩阵的条件数；未计算或秩为 0 时为 0。
  double scaled_condition_number = 0.0;

  bool success = false;   ///< 是否求解成功
  std::string message;    ///< 失败原因（中文、可直接打印）
};

/**
 * @brief 由回归矩阵与实测力矩求解惯性参数。
 * @param regressor [in] 堆叠回归矩阵 W，行 = 采样点数 × 关节数，列 = 10 × 段数。
 * @param torque    [in] 实测力矩向量 τ_meas，长度须 = W.rows()，行块顺序与 W 一致。
 * @param options   [in] 求解选项，默认值对"解析构造的无噪声数据"友好。
 * @return IdentificationResult；success == false 时 message 说明原因。
 *
 * @note 行块顺序约定与 kdl_regressor::buildRegressor() 一致：第 i 个采样点的
 *       关节力矩连续 n 个元素排在一起。
 * @note 本函数**不做**任何物理性检查（质量是否为正、惯量矩阵是否正定等），
 *       那些属于"辨识后处理"，应由调用者结合工程知识判断。
 */
IdentificationResult solveLeastSquares(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & torque,
  const LeastSquaresOptions & options = LeastSquaresOptions());

}  // namespace kdl_identification

#endif  // KDL_PARAMETER_IDENTIFICATION__KDL_LSQ_HPP_
