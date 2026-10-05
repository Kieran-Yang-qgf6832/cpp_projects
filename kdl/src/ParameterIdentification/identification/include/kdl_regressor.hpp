// Copyright (c) 2026, kdl_identification authors.
// 教学用途：机械臂惯性参数辨识的"回归矩阵"（regressor）。
//
// ===========================================================================
// 一、为什么可以"辨识"
// ===========================================================================
// 刚体机械臂的逆动力学对**惯性参数是严格线性**的。把每个连杆的 10 个标准
// 惯性参数（相对该连杆参考系原点）排成一个向量：
//
//     β_L = [ m, m·c_x, m·c_y, m·c_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz ]ᵀ
//              ~     ~~~~~~~ 第一阶矩(m·c) ~~~~~~~   ~~~ 转动惯量 ~~~
//
// 那么"给定运动，求所需关节力矩"这件事可以写成：
//
//     τ = Y(q, q̇, q̈) · β
//
// 其中 τ ∈ R^n（n = 自由度数），β ∈ R^(10·段数)。Y 就是回归矩阵：它只由
// 运动 (q, q̇, q̈) 和连杆几何/关节轴决定，**完全不含质量与惯量**。
// 整个辨识问题因此变成一个线性最小二乘：采一堆 (q, q̇, q̈, τ)，堆出一个
// 超定方程 W β = τ，解它即可（见 kdl_lsq.hpp）。
//
// ===========================================================================
// 二、KDL 没有 regressor，怎么得到 Y
// ===========================================================================
// orocos-kdl 只提供"给定 β 求 τ"的 inverseDynamics，没有给出 Y。手写
// Newton-Euler 的线性化（把每个 10×10 参数矩阵摊开）既长又容易错。
//
// 本模块利用上面那条**线性性质**，用"精确差分"来构造 Y：
//
//     Y 的第 k 列 = ID(β₀ + e_k) − ID(β₀)
//
// 因为 ID 对 β 是线性的，这个差分**不是近似**——对任意步长都严格成立
// （步长取 1，即该参数增加一个单位）。代价只是"多调几次 inverseDynamics"，
// 这对离线辨识完全可以接受（P = 10·段数，本臂 6 段即 60 次）。
//
// 唯一的技术点：要让 KDL 按任意 β 求值，必须能从 (m, h, I_原点) 重建
// RigidBodyInertia。KDL 的公开构造函数只接受 COG 位置的惯量，
// 所以 applyParameters() 里做一次平行轴定理的换算。
//
// ===========================================================================
// 三、坐标系约定（很重要）
// ===========================================================================
// β 是相对**每段自己的参考系原点**（KDL 里就是该段的关节坐标系）定义的，
// 不是相对 URDF <inertial> 里那个带 origin 偏置的 link 坐标系。
// kdl_parser 在建模时已经把 URDF 的惯量折算到了关节坐标系，所以本模块
// extractParameters() 读出来的 β 和后面辨识出来的 β 自洽，
// 但与 URDF 文件里写的数字**不直接相等**（差一个平移）。
//
// 另外：固定段（KDL::Joint::None）的参数不影响任何关节力矩，对应列恒为 0。
// 这不是 bug，是"不可辨识参数"，由 kdl_lsq 的 QR/SVD 自动识别并剔除。

#ifndef KDL_PARAMETER_IDENTIFICATION__KDL_REGRESSOR_HPP_
#define KDL_PARAMETER_IDENTIFICATION__KDL_REGRESSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_idynamics.hpp"  // kdl_dynamics::defaultGravity()

namespace kdl_identification
{

// ---------------------------------------------------------------------------
// 一、参数向量的布局
// ---------------------------------------------------------------------------

/// 每个连杆的标准惯性参数个数（质量 1 + 第一阶矩 3 + 转动惯量 6）。
constexpr unsigned int kParamsPerLink = 10;

/**
 * @brief 返回一个连杆块内第 index 个分量的名字（用于打印/诊断）。
 * @param index [in] 块内下标 ∈ [0, 9]。
 * @return 固定字符串；越界返回 "?"。
 *
 * @note 顺序：0=m, 1..3=m·cx/m·cy/m·cz, 4..9=Ixx/Ixy/Ixz/Iyy/Iyz/Izz。
 *       这个顺序在 extractParameters / applyParameters / 回归矩阵列之间
 *       必须完全一致，集中在这里定义，避免各处写死数字。
 */
const char * parameterName(unsigned int index);

/**
 * @brief 参数向量长度 = 10 × 链的段数。
 * @param chain [in] 运动学链。
 * @return 参数个数（含固定段；固定段参数不可辨识，但列仍占位）。
 */
unsigned int parameterCount(const KDL::Chain & chain);

/**
 * @brief 生成人类可读的参数标签，例如 "link3.Ixx"。
 * @param chain [in] 运动学链。
 * @param index [in] 参数总下标 ∈ [0, parameterCount() − 1]。
 * @return "段名.分量名"；越界返回 "<invalid>"。
 *
 * @note 诊断时用它可以一眼看出"哪些参数被识别为基参数、值是多少"。
 */
std::string parameterLabel(const KDL::Chain & chain, unsigned int index);

// ---------------------------------------------------------------------------
// 二、参数 <-> 链 的相互转换
// ---------------------------------------------------------------------------

/**
 * @brief 从一条 KDL 链读出其当前的惯性参数向量 β。
 * @param chain      [in]  运动学链（各段的 RigidBodyInertia 即真值来源）。
 * @param parameters [out] 长度 parameterCount(chain) 的参数向量。
 * @param message    [out] 失败原因；成功时清空。
 * @return true 表示成功。
 *
 * @note 质量 m ≈ 0 的段无法从 KDL 中还原第一阶矩（KDL 存的是 m·c，
 *       getCOG() 在 m=0 时直接返回 0），此时本函数把该段的第一阶矩记为 0，
 *       这是唯一的信息损失；真实的机械臂连杆质量都远大于 0。
 */
bool extractParameters(
  const KDL::Chain & chain, Eigen::VectorXd & parameters, std::string & message);

/**
 * @brief 用给定的惯性参数向量重建一条链（关节、几何、力矩常量都不变）。
 * @param chain      [in]  模板链（决定关节类型/轴/段偏移/段名）。
 * @param parameters [in]  长度 parameterCount(chain) 的参数向量。
 * @param result     [out] 重建后的链；失败时保持原样。
 * @param message    [out] 失败原因；成功时清空。
 * @return true 表示成功。
 *
 * @note 内部对每个连杆做一次平行轴定理换算：KDL 的构造函数要的是
 *       **COG 处**的转动惯量，而 β 给的是**参考点处**的，两者相差
 *       m·(|c|²·Id − c·cᵀ)（c 为参考点到质心的向量，即 β 的 m·c / m）。
 * @note m ≤ 0 的段无法用上述公式（要除以 m），除非第一阶矩恰为 0；
 *       后者按"质心在参考点"处理。物理上不应出现负质量。
 */
bool applyParameters(
  const KDL::Chain & chain, const Eigen::VectorXd & parameters, KDL::Chain & result,
  std::string & message);

// ---------------------------------------------------------------------------
// 三、回归矩阵
// ---------------------------------------------------------------------------

/**
 * @brief 一个采样点的运动状态（辨识的最小数据单元）。
 *
 * @note 三者单位：弧度、rad/s、rad/s²（平移关节换成 m 系）。
 *       实际采集时 q̇ 直接来自编码器/仿真，q̈ 一般由 q̇ 差分加低通得到。
 */
struct RegressorSample
{
  KDL::JntArray q;      ///< 关节角 [rad]
  KDL::JntArray qdot;   ///< 关节速度 [rad/s]
  KDL::JntArray qddot;  ///< 关节加速度 [rad/s²]
};

/**
 * @brief 回归矩阵的构建结果。
 *
 * @note 与 kdl_interpolation 的 TrajectoryResult 同一体例：把"能不能用"
 *       和"为什么不能用"一起交给调用者，不用异常打断离线计算。
 */
struct RegressorResult
{
  /// 回归矩阵：行 = 关节数 × 采样点数（按采样点分块），列 = 10 × 段数。
  Eigen::MatrixXd regressor;

  unsigned int joints = 0;    ///< 自由度数 n
  unsigned int segments = 0;  ///< 链的段数
  unsigned int samples = 0;   ///< 采样点数 N

  bool success = false;       ///< 是否构建成功
  std::string message;        ///< 失败原因（中文、可直接打印）

  /**
   * @brief 行数与参数向量的对应关系。
   * @return 行数 = joints × samples。
   */
  unsigned int rows() const { return joints * samples; }
};

/**
 * @brief 可复用的回归矩阵构造器：把"造 10·段数 + 1 条链"的开销放在构造时。
 *
 * @note 回归矩阵的每一列都是"某条参数被单位扰动的链"上的逆动力学结果，只由
 *       采样点决定、与辨识结果无关。激励轨迹优化、批量/在线辨识这类要对同一台
 *       机械臂**反复构建**回归矩阵的场景，用本类构造一次、反复 build() 能省下
 *       大量重复建链开销。只做一次辨识时，直接用下面的自由函数即可。
 */
class RegressorBuilder
{
public:
  /**
   * @param chain   [in] 运动学链。
   * @param gravity [in] 重力向量（基座系）。
   * @note 构造失败（链为空、参数无法表示）时 valid() 为 false，error() 给出原因，
   *       此时 build() 会直接返回失败结果而不会崩溃。
   */
  explicit RegressorBuilder(
    const KDL::Chain & chain, const KDL::Vector & gravity = kdl_dynamics::defaultGravity());

  /// @return 构造是否成功。
  bool valid() const { return valid_; }

  /// @return 构造失败的原因（成功时为空）。
  const std::string & error() const { return error_; }

  /// @return 自由度数。
  unsigned int joints() const { return joints_; }

  /// @return 链的段数。
  unsigned int segments() const { return segments_; }

  /// @return 参数向量长度 = 10 × 段数。
  unsigned int parameterCount() const { return kParamsPerLink * segments_; }

  /// @return 名义链（用 extractParameters 的目标参数重建的那条链）。
  const KDL::Chain & nominalChain() const { return nominal_chain_; }

  /**
   * @brief 对单个采样点构建回归矩阵 Y。
   * @param q       [in]  关节角 [rad]。
   * @param qdot    [in]  关节速度 [rad/s]。
   * @param qddot   [in]  关节加速度 [rad/s²]。
   * @param Y       [out] n × 10·段数 的回归矩阵。
   * @param message [out] 失败原因。
   * @return true 表示成功。
   */
  bool build(
    const KDL::JntArray & q, const KDL::JntArray & qdot, const KDL::JntArray & qddot,
    Eigen::MatrixXd & Y, std::string & message) const;

  /**
   * @brief 对一批采样点构建堆叠回归矩阵。
   * @param samples [in] 采样点序列。
   * @return RegressorResult，行数 = n × samples。
   */
  RegressorResult build(const std::vector<RegressorSample> & samples) const;

  // 本类内部持有的 KDL 求解器**引用**着两条链（nominal_chain_ / basis_chains_）。
  // 一旦拷贝或搬移，求解器引用的就是旧地址（已析构或已搬空的对象），
  // CartToJnt 会报 "Internal data structures not up to date with Chain"。
  // 所以禁止拷贝与移动：需要共享时请用 std::unique_ptr 或引用。
  RegressorBuilder(const RegressorBuilder &) = delete;
  RegressorBuilder & operator=(const RegressorBuilder &) = delete;
  RegressorBuilder(RegressorBuilder &&) = delete;
  RegressorBuilder & operator=(RegressorBuilder &&) = delete;

private:
  // 成员声明顺序 = 构造顺序，析构顺序相反。KDL 的 ChainIdSolver_RNE 内部持有
  // 对链的**引用**，所以 solvers_ 必须声明在两条链之后 —— 这样它会先被析构。
  bool valid_ = false;                  ///< 构造是否成功
  std::string error_;                   ///< 构造失败原因
  KDL::Chain nominal_chain_;            ///< 名义链
  std::vector<KDL::Chain> basis_chains_;  ///< 每列一条"单位扰动"链
  std::vector<std::unique_ptr<KDL::ChainIdSolver_RNE>> solvers_;  ///< [0]=名义，[k+1]=第 k 列
  KDL::Vector gravity_;                 ///< 重力向量
  unsigned int joints_ = 0;             ///< 自由度数
  unsigned int segments_ = 0;           ///< 段数

  // build() 是 const，但求解需要可写的"草稿纸"，所以这些缓存是 mutable 的。
  // 反复用它避免每个采样点都重新分配 KDL 的内部缓冲（这是性能关键）。
  mutable KDL::Wrenches external_wrenches_;  ///< 空外力（长度 = 段数，全零）
  mutable KDL::JntArray nominal_torque_;     ///< 名义链的力矩输出缓冲
  mutable KDL::JntArray column_torque_;      ///< 扰动链的力矩输出缓冲
};

/**
 * @brief 对单个采样点构建回归矩阵 Y（n × 10·段数），满足 τ = Y·β。
 * @param chain   [in] 运动学链。
 * @param q       [in] 关节角 [rad]，长度须 = chain.getNrOfJoints()。
 * @param qdot    [in] 关节速度 [rad/s]。
 * @param qddot   [in] 关节加速度 [rad/s²]。
 * @param gravity [in] 重力向量（基座系）；默认 (0, 0, −9.81)。
 * @return RegressorResult；samples = 1，regressor 为 n × 10·段数。
 *
 * @note 构造方式见文件头"二"：对每个参数做一次单位差分。构造过程会临时
 *       重建 10·段数 + 1 条链，所以对"控制环内每周期调用"不合适，
 *       它是**离线辨识**用的接口。
 */
RegressorResult buildRegressor(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Vector & gravity = kdl_dynamics::defaultGravity());

/**
 * @brief 对一批采样点构建**堆叠**回归矩阵 W，满足 τ_vec = W·β。
 * @param chain   [in] 运动学链。
 * @param samples [in] 采样点序列，长度 ≥ 1；每点的三个数组长度须一致且 = n。
 * @param gravity [in] 重力向量（基座系）；默认 (0, 0, −9.81)。
 * @return RegressorResult；行数 = n × samples。
 *
 * @note 行块顺序与"测量力矩向量"必须一致：
 *       第 i 个采样点的 n 行连续排布，对应 τ_meas 里第 i 段 n 个元素。
 *       kdl_lsq::solveLeastSquares() 直接吃这个 W 与同样堆叠的 τ_vec。
 */
RegressorResult buildRegressor(
  const KDL::Chain & chain, const std::vector<RegressorSample> & samples,
  const KDL::Vector & gravity = kdl_dynamics::defaultGravity());

}  // namespace kdl_identification

#endif  // KDL_PARAMETER_IDENTIFICATION__KDL_REGRESSOR_HPP_
