// Copyright (c) 2026, kdl_identification authors.
// 教学用途：带 LMI 约束的参数求解 —— 把"拟合 τ"和"物理可行"同时满足。
//
// ===========================================================================
// 一、要解的问题
// ===========================================================================
// 辨识出来的 β_id 只满足 W·β = τ，不保证物理可行；而"喂给控制器/仿真"要求
// 每一段的伪惯量矩阵必须半正定。于是要求解
//
//     min ‖S·(β − β_prior)‖
//     s.t.  W·β = W·β_id          （仿射集 A：复现实测力矩）
//           J_s(β) ⪰ 0   ∀段 s     （谱面体 P：物理可行）
//
// 其中伪惯量矩阵
//
//           ⎡ ½·tr(I_ref)·Id − I_ref    h ⎤
//     J(β) =⎢                              ⎥ ,   β = (m, h, I_ref)
//           ⎣           hᵀ               m ⎦
//
// 它等于质量密度 ρ(r) 的二阶矩 ∫ρ(r)·[r;1][r;1]ᵀ dV，所以 J ⪰ 0 恒成立、
// J ≻ 0 当且仅当这组参数真能由某个三维质量分布实现（等价于 m>0 且 I_cog ≻ 0）。
// 关键是 **J 对 β 是线性的**，所以这是个标准 SDP：目标二次、约束是 LMI。
//
// ===========================================================================
// 二、为什么不用"投影 + 退让"那套启发式
// ===========================================================================
// 仿射集 A 里存在"高杠杆"方向：τ 上极小的差异会映射成 β 上巨大的位移。
// 实测（真实闭环数据）：力矩残差只有 4.5e-3 N·m，但最小二乘解离先验 312.8。
// 于是"离先验最近的点"恰好落在可行域 P 外面，而沿"先验→该点"退让只能退到
// t≈0（等于直接用先验），识别出的信息全部丢掉。
// 正确做法是**在可行域内部**找最优点，而不是在可行域外找最近点再往回退。
//
// ===========================================================================
// 三、算法：移位对数障碍 + Newton（自写，不引入 SDP 求解器依赖）
// ===========================================================================
// 问题规模很小（本臂：18 个自由变量、4 个 4×4 的 LMI），所以直接上障碍函数法：
//
//     Φ(z) = φ(z) + μ·[ −Σ_s log det( J_s(z) + σ·Id ) ]
//     φ(z) = ½‖S·(β_id + N·z − β_prior)‖²
//
// · 用零空间参数化 β = β_id + N·z 把等式约束消掉（N 是 null(W) 的基）；
// · J_s(z) 对 z 仿射（J 对 β 线性），所以梯度/海塞都能闭式写出来；
// · σ 是**移位量**：它让障碍在初始点也有定义（初始点 β_id 通常不可行），
//   外层把 (σ, μ) 同步缩小，σ→0 时障碍把迭代点逼进 P 的内部；
// · 若 A ∩ int(P) 非空，迭代收敛到约束最优；若为空，σ 缩不下去，报不可行。
//
// 这条路径**不保证**比专业 SDP 求解器稳健（没有预处理、没有 Mehrotra 校正），
// 但它是自包含的，且正确性可以用"已知答案"的场景验证（见 export_urdf_demo）。

#ifndef KDL_PARAMETER_IDENTIFICATION__KDL_LMI_PROJECTION_HPP_
#define KDL_PARAMETER_IDENTIFICATION__KDL_LMI_PROJECTION_HPP_

#include <string>

#include <Eigen/Core>

namespace kdl_identification
{

/**
 * @brief LMI 约束下求解的结果。
 */
struct LmiSolveResult
{
  bool success = false;              ///< 是否收敛到可行且满足最优性
  std::string message;               ///< 失败原因（中文、可直接打印）

  /// 完整长度的参数向量（长度 = W 的列数）。
  Eigen::VectorXd parameters;

  unsigned int variables = 0;        ///< 自由变量个数 = 零空间维数
  unsigned int outer_iterations = 0; ///< 外层（σ/μ 递减）迭代次数
  unsigned int newton_steps = 0;     ///< 内层 Newton 步总数

  /// ‖S·(β − β_prior)‖：目标函数值（越小越接近先验）。
  double scaled_distance = 0.0;

  /// 最优点的最小伪惯量特征值（> 0 表示严格可行；≈0 表示贴边）。
  double min_eigenvalue = 0.0;

  /// 最后的移位量 σ；收敛时应接近 0。若明显大于 0，说明 A ∩ int(P) 可能为空。
  double final_shift = 0.0;

  /// 实际采用的自由变量解 z（诊断用；β = β_id + N·z）。
  Eigen::VectorXd z;

  /// 是否做过"贴边修复"（把 1e-9 量级的数值违约夹掉）。
  bool boundary_repaired = false;

  /// 修复前后参数的改变量 ‖Δβ‖（应远小于 ‖β‖）。
  double repair_norm = 0.0;
};

/**
 * @brief 把一组"差一点点就可行"的参数夹成**严格**物理可行。
 * @param parameters [in] 长度须为 10 的整数倍。
 * @return 修复后的参数。
 *
 * @note 逐段把质心惯量 I_cog 的特征值夹到 ≥ 一个极小正数，并满足主惯量三角不等式，
 *       再用同一个平行轴项换回 I_ref。
 * @note 这会**轻微**破坏 W·β = W·β_id。仅当违约在数值噪声量级（实测 1e-9 相对量级）
 *       时才该用它 —— 此时对力矩的影响完全可以忽略。调用方应把改变量
 *       （LmiSolveResult::repair_norm）如实报出来。
 */
Eigen::VectorXd repairToStrictFeasibility(const Eigen::VectorXd & parameters);

/**
 * @brief 求解 min ‖S·(β − β_prior)‖ s.t. W·β = W·β_id, J_s(β) ⪰ 0。
 * @param regressor     [in] 回归矩阵 W，列数须为 10 的整数倍。
 * @param identified    [in] 最小二乘解 β_id（长度 = W 的列数）。
 * @param prior         [in] 先验参数 β_prior（长度 = W 的列数）。
 * @param scale         [in] 度量尺度 S 的对角元（长度 = 列数；留空则全 1）。
 * @param rank_tolerance[in] 判秩相对阈值（决定零空间维数）。
 * @return LmiSolveResult。
 *
 * @note 先验 β_prior **不要求**在仿射集里（现实里它就不在）。它只提供度量中心。
 * @note 判秩用列归一化后的 W（与 kdl_lsq / completeWithPrior 一致）。
 */
LmiSolveResult solveLmiConstrainedCompletion(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & identified,
  const Eigen::VectorXd & prior, const Eigen::VectorXd & scale = Eigen::VectorXd(),
  double rank_tolerance = 1e-9);

/**
 * @brief 计算一组参数的伪惯量矩阵（每段一个 4×4）。
 * @param block [in] 长度 10 的参数块：(m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz)。
 * @return 4×4 伪惯量矩阵 J。
 *
 * @note 单独暴露出来是为了让调用方/测试能直接检查 J 的特征值。
 */
Eigen::Matrix4d pseudoInertia(const Eigen::VectorXd & block);

}  // namespace kdl_identification

#endif  // KDL_PARAMETER_IDENTIFICATION__KDL_LMI_PROJECTION_HPP_
