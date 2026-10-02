// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：把逆动力学的"三明治公式"拆成三个可以单独研究的量。
//
//     τ = M(q)·q̈  +  C(q,q̇)·q̇  +  G(q)
//
// 这三项各有明确的物理含义，分开看比揉在一起更有教学价值：
//
//   M(q)       关节空间惯性矩阵。它描述"每个关节加速的难易程度以及关节之间的
//              耦合"。物理上必然**对称**且**正定**（本文件提供了检查函数）。
//   C(q,q̇)·q̇   科氏力与离心力项。它只在关节运动时出现（q̇ = 0 时为零），
//              源于连杆之间的相对运动与坐标系旋转。
//   G(q)       重力项。即使机器人完全静止（q̇ = q̈ = 0），关节也要出这个力矩
//              才能保持不动，这就是"重力补偿"要抵消的部分。
//
// KDL 里对应的求解器是 ChainDynParam，它内部其实是用 RNE 反复调用来算的：
// 算 G 用 (q̇=0, q̈=0)，算 C 用 (q̈=0 且关掉重力)。

#ifndef KDL_DYNAMICS__KDL_DYNPARAM_HPP_
#define KDL_DYNAMICS__KDL_DYNPARAM_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include "kdl_idynamics.hpp"

namespace kdl_dynamics
{

// ---------------------------------------------------------------------------
// 一、结果结构
// ---------------------------------------------------------------------------

/**
 * @brief 关节空间惯性矩阵 M(q) 的计算结果。
 */
struct MassResult
{
  KDL::JntSpaceInertiaMatrix mass;  ///< M(q)，对称正定矩阵：对角项单位 kg·m²，耦合项 kg·m²
  int error_code = 0;               ///< KDL 错误码：0=正常，负数=失败
  std::string message;              ///< 错误码对应的可读说明

  /// @return error_code >= 0 时返回 true。
  bool success() const { return error_code >= 0; }
};

/**
 * @brief 科氏/离心项 C(q,q̇)·q̇ 的计算结果。
 */
struct CoriolisResult
{
  KDL::JntArray torque;   ///< 各关节上的科氏/离心力矩，单位 N·m（平移关节为 N）
  int error_code = 0;     ///< KDL 错误码
  std::string message;    ///< 错误码对应的可读说明

  /// @return error_code >= 0 时返回 true。
  bool success() const { return error_code >= 0; }
};

/**
 * @brief 重力项 G(q) 的计算结果。
 */
struct GravityResult
{
  KDL::JntArray torque;   ///< 各关节上的重力力矩，单位 N·m（平移关节为 N）
  int error_code = 0;     ///< KDL 错误码
  std::string message;    ///< 错误码对应的可读说明

  /// @return error_code >= 0 时返回 true。
  bool success() const { return error_code >= 0; }
};

// ---------------------------------------------------------------------------
// 二、三项分解
// ---------------------------------------------------------------------------

/**
 * @brief 计算关节空间惯性矩阵 M(q)。
 * @param chain   [in] 运动学链。
 * @param q       [in] 关节角，单位 rad（平移关节为 m）。
 * @param gravity [in] 重力向量，基座坐标系，单位 m/s²；默认 (0, 0, -9.81)。
 * @return MassResult，其中 mass 为 n x n 的惯性矩阵（n = 自由度数）。
 *
 * @note 惯性矩阵**只依赖位形 q**，与 q̇、q̈ 无关；位形一变它就要重算。
 * @note 本函数接受 gravity 只是为了让接口与其它函数保持一致 ——
 *       严格来说 M(q) 并不含重力，重力不影响惯性矩阵。
 */
MassResult jointSpaceInertia(
  const KDL::Chain & chain, const KDL::JntArray & q,
  const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 计算科氏/离心项 C(q,q̇)·q̇。
 * @param chain   [in] 运动学链。
 * @param q       [in] 关节角，单位 rad。
 * @param qdot    [in] 关节速度，单位 rad/s。
 * @param gravity [in] 重力向量，基座坐标系，单位 m/s²；默认 (0, 0, -9.81)。
 * @return CoriolisResult，其中 torque 为科氏/离心力矩，单位 N·m。
 *
 * @note 这一项在 q̇ = 0 时恒为零 —— 示例中可以直接验证这个性质。
 * @note 注意它返回的是**已经乘过 q̇ 的力矩向量** C(q,q̇)·q̇，
 *       而不是矩阵 C(q,q̇) 本身（KDL 只提供前者）。
 */
CoriolisResult coriolisTorque(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 计算重力项 G(q)。
 * @param chain   [in] 运动学链。
 * @param q       [in] 关节角，单位 rad。
 * @param gravity [in] 重力向量，基座坐标系，单位 m/s²；默认 (0, 0, -9.81)。
 * @return GravityResult，其中 torque 为重力力矩，单位 N·m。
 *
 * @note **返回值本身就是"重力补偿力矩"**：即执行器为抵住重力所需施加的力矩，
 *       控制器**直接施加 τ = +G(q)** 即可在任意位形静止平衡，**不要取负**
 *       （取负等于把重力加倍）。已用一维摆判据实测确认：
 *       ID(q,0,0) == G(q)，且 FD(τ = +G(q)) → q̈ = 0（静止）。
 *       这条与 inverseDynamics(chain, q, 0, 0) 完全同号同值，两者可互相印证。
 * @note 若把 gravity 设为 Vector::Zero()，这里会返回全零，
 *       可以用来单独观察"无重力"环境下 M 与 C 的行为。
 */
GravityResult gravityTorque(
  const KDL::Chain & chain, const KDL::JntArray & q,
  const KDL::Vector & gravity = defaultGravity());

// ---------------------------------------------------------------------------
// 三、惯性矩阵的物理性质检查
// ---------------------------------------------------------------------------

/**
 * @brief 检查惯性矩阵是否对称（M = Mᵀ）。
 * @param mass 待检查的惯性矩阵 M(q)。
 * @param tol  判定容差，默认 1e-9；|M(i,j) − M(j,i)| <= tol 视为相等。
 * @return 对称返回 true。
 *
 * @note 对称性来自物理本质（动能可以对 q̇ 求两次导数，与顺序无关），
 *       所以这既是数值检查，也是对 KDL 求解器的一个健全性验证。
 */
bool isSymmetric(const KDL::JntSpaceInertiaMatrix & mass, double tol = 1e-9);

/**
 * @brief 检查惯性矩阵是否正定。
 * @param mass 待检查的惯性矩阵 M(q)。
 * @return 正定返回 true。
 *
 * @note 正定的物理含义：任何非零关节加速度都必须消耗正的能量，
 *       也就是说机器人不可能"零力矩自己加速起来"。
 * @note 实现上用 Cholesky 分解（Eigen::LLT）判断：分解成功即为正定。
 */
bool isPositiveDefinite(const KDL::JntSpaceInertiaMatrix & mass);

}  // namespace kdl_dynamics

#endif  // KDL_DYNAMICS__KDL_DYNPARAM_HPP_
