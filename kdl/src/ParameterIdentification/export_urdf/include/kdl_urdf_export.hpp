// Copyright (c) 2026, kdl_identification authors.
// 教学用途：把辨识出的惯性参数写回 URDF 的 <inertial>，并做零空间补全。
//
// ===========================================================================
// 一、为什么需要"导出"
// ===========================================================================
// kdl_regressor 里的 β 是**相对每段关节坐标系原点**定义的：
//
//     β_L = [ m, m·cx, m·cy, m·cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz ]ᵀ
//
// 而 URDF 的 <inertial> 写的是**相对 link 坐标系**、且惯量是**相对质心**的：
//
//     <origin xyz="质心位置"/>  <mass value="m"/>  <inertia .../>（质心惯量）
//
// 两者差一个平行轴定理。本模块负责这个换算，并生成一份能被 kdl_parser /
// robot_state_publisher / 控制器 `urdf_file` 参数直接读取的新 URDF。
//
// 已实证确认的对应关系（探针输出见 README）：
//   · KDL 链段的名字 == URDF 的 child link 名（本臂：link1..link4）
//   · KDL 链段的参考系 == 该 link 的坐标系（原点与姿态都一致）
//   · RigidBodyInertia::getRotationalInertia() 返回**相对参考点**的惯量
//     （KDL 头文件原文："expressed in the reference frame (not the cog)"），
//     所以 β 的惯量块 = URDF <inertia> + m·(|c|²·Id − c·cᵀ)
//
// ===========================================================================
// 二、什么参数"能"写进 URDF —— 为什么必须先补全
// ===========================================================================
// URDF 的 <inertial> 只能表达 m > 0 的刚体。而 KDL 的 RigidBodyInertia 内部
// 存的是第一阶矩 h = m·c，构造函数里 h 恒等于 m·c —— 所以 "m ≈ 0 且 h ≠ 0"
// 这个组合在 KDL 与 URDF 里都**无解**（kdl_regressor 的 applyParameters 会
// 直接报错返回 false）。
//
// 麻烦在于：最小二乘的结果恰恰会把**不可辨识的参数置 0**（见 kdl_lsq）。
// 串联臂里 "第一连杆的质量 m" 通常结构性不可辨识，于是 β_id 里就出现
// m = 0 而 m·cx ≠ 0 的非法组合，**直接导出必然失败**。
//
// 对策是 completeWithPrior()：在所有满足 W·β = W·β_id 的解里，挑"最接近先验
// （URDF 原值）"的那一个。它只改动**零空间方向**（对任何运动都不影响 τ），
// 因而既保住了测量信息、又让参数重新变得物理可实现。
//
// ===========================================================================
// 三、导出方式：文本外科替换
// ===========================================================================
// 不做 XML 库的"解析-重排-序列化"，而是只把每个 <link> 里的 <inertial>
// 整块换掉，其余字节原样保留。好处是 diff 极小（只有惯量数字变了），
// 原 URDF 的注释、缩进、mesh 路径全部不受影响，便于人工核对。
// 代价是标签要自己找，所以配套提供 verifyExportedUrdf() 做往返校验：
// 重新解析导出的 URDF，比对参数是否与写出去的一致（不一致就说明文本操作错了）。

#ifndef KDL_PARAMETER_IDENTIFICATION__KDL_URDF_EXPORT_HPP_
#define KDL_PARAMETER_IDENTIFICATION__KDL_URDF_EXPORT_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>

namespace kdl_identification
{

// ---------------------------------------------------------------------------
// 一、参数补全（导出的前置步骤）
// ---------------------------------------------------------------------------

/**
 * @brief 逐段的物理可行性报告。
 */
struct PhysicalFeasibility
{
  bool feasible = false;                 ///< 是否全部段都物理可行
  std::string message;                   ///< 失败原因（指出段号与量值）
  unsigned int segments = 0;             ///< 段数
  double min_mass = 0.0;                 ///< 各段质量的最小值（须 > 0）
  double min_inertia_eigenvalue = 0.0;   ///< 各段质心惯量最小特征值（须 > 0）
  unsigned int worst_segment = 0;        ///< 最先出问题的段号
};

/**
 * @brief 检查一组惯性参数是否物理可行。
 * @param parameters [in] 长度须为 10 的整数倍（m, h, I_ref 的布局）。
 * @return PhysicalFeasibility。
 *
 * @note 判据（三者都是刚体惯量的必要条件）：
 *         1) 质量 m > 0；
 *         2) 质心惯量 I_cog = I_ref − m·(|c|²·Id − c·cᵀ) 正定（最小特征值 > 0）；
 *         3) 主惯量的三角不等式 Ixx+Iyy ≥ Izz（轮换）。
 *
 * @note 为什么需要这个检查：辨识得到的参数满足的是"τ 一致"，而**物理可行集**
 *       （伪惯量矩阵 J(β) = [[½·tr(I_ref)·Id − I_ref, h], [hᵀ, m]] ≻ 0，是一个
 *        线性矩阵不等式 LMI；等价于 m > 0 且 I_cog ≻ 0）
 *       与"τ 等价的仿射集"并不重合。零空间补全只是解一个二次规划，**不保证**
 *       落在可行集里 —— 实测中 link3 会导出负的对角惯量（它的惯量 ~1e-4，
 *       而质心被零空间推到 0.26 m 后平行轴项达 ~5.7e-3）。
 *       严谨的修法是带 LMI 约束的凸优化（SDP），本模块不引入该依赖，改为
 *       提供"检查 + 退让"这条简单可靠的路径（见 CompletionOptions）。
 */
PhysicalFeasibility checkPhysicalFeasibility(const Eigen::VectorXd & parameters);

/**
 * @brief 零空间补全的选项。
 */
struct CompletionOptions
{
  /// 判秩相对阈值，与 LeastSquaresOptions::rank_tolerance 同义。
  double rank_tolerance = 1e-9;

  /**
   * 各参数的"典型量级" s_j（长度须 = 列数；留空则自动按先验推断）。
   *
   * 补全在这个度量下最小化 ‖S·(β − β_prior)‖，也就是"**相对**先验偏差最小"。
   * 之所以不能用欧氏度量（s_j ≡ 1，即绝对偏差最小）：β 里 m（kg）、m·c（kg·m）、
   * I（kg·m²）的量纲差好几个数量级，零空间里存在"沿关节轴平移质心"这类
   * 高杠杆方向 —— 它在欧氏度量下几乎不花代价，却能把质心推到几十米外。
   * 实测（见 README）：同一组数据下，欧氏度量导出的最大 |质心| 明显更大
   * （噪声 0.02 N·m 时 0.557 m vs 本度量 0.258 m），而力矩拟合精度不变。
   *
   * 留空时的自动推断：s_j = max(|β_prior_j|, 1e-3 × 同类型参数的最大绝对值)。
   * 后一项是下限，避免先验里恰好为 0 的分量把 s_j 变成 0。
   */
  Eigen::VectorXd scale;

  /**
   * 是否保证补全结果物理可行。
   *
   * 开启后若补全结果不可行，就在"可行集 ↔ 仿射集"之间做交替投影（POCS，
   * 见 .cpp 里的说明）：两集都是凸的，若相交则收敛到交集中的一点 ——
   * 那种情况下**既**精确拟合 τ **又**物理可行，是严格意义上的解。
   * 若两集不相交（噪声把仿射集整体推离可行集时会发生），则退回
   * "先验 → 补全结果"线段上可行的最大比例 t，并把代价如实记在
   * CompletionResult::torque_residual_norm 里。
   *
   * 这是一条**工程路径**而不是严格解：严格解要在 LMI 约束下做凸优化（SDP），
   * 本模块不引入该依赖。关掉它则一定精确满足 W·β = W·β_id（力矩拟合最好），
   * 但导出物可能非物理（实测中确实会出现负的对角惯量）。
   */
  bool enforce_physical_feasibility = false;

  /// 交替投影的迭代次数（仅在上面开关打开时使用）。
  unsigned int feasibility_iterations = 400;
};

/**
 * @brief 零空间补全的结果。
 */
struct CompletionResult
{
  /// 补全后的完整参数向量，长度 = W 的列数。
  Eigen::VectorXd parameters;

  unsigned int rank = 0;               ///< W 的数值秩（可辨识基参数个数）
  unsigned int null_space_dimension = 0;  ///< 零空间维数 = 列数 − 秩

  /// ‖β_补全 − β_先验‖₂：补全结果离先验多远（绝对值）。
  double correction_norm = 0.0;

  /// ‖β_辨识 − β_先验‖₂：补全前的原始差距。两者之比 ≤ 1，越小说明搬回先验越多。
  double prior_gap_norm = 0.0;

  /// 实际采用的退让比例 t ∈ [0, 1]：1 表示无需退让（未开启可行性约束时恒为 1）。
  double feasibility_fraction = 1.0;

  /// ‖W·β_补全 − W·β_id‖₂：补全结果与原辨识解在**力矩上**的偏差。
  /// 不退让时为 0（或数值噪声级）；退让后这个数就是"为了物理可行付出的代价"。
  double torque_residual_norm = 0.0;

  /// 补全结果是否通过 checkPhysicalFeasibility（未检查时为 false）。
  bool physically_feasible = false;

  bool success = false;   ///< 是否成功
  std::string message;    ///< 失败原因（中文、可直接打印）
};

/**
 * @brief 在"与辨识结果等价"的解里，挑**相对先验偏差最小**的那一个。
 * @param regressor  [in] 回归矩阵 W，行 = 采样点数 × 关节数，列 = 10 × 段数。
 * @param identified [in] 最小二乘解 β_id（长度 = W 的列数）。
 * @param prior      [in] 先验参数 β_prior（URDF 原值；长度 = W 的列数）。
 * @param options    [in] 选项（判秩阈值 + 度量尺度）。
 * @return CompletionResult。
 *
 * @note 数学上要解的是
 *           min ‖S·(β − β_prior)‖    s.t.  W·β = W·β_id
 *       其中 S = diag(s_j) 是 options.scale。作变量替换 γ = S·(β − β_prior)，
 *       约束变成 A·γ = r（A = W·S⁻¹，r = W·(β_id − β_prior)），于是
 *           γ = A⁺·r（截断伪逆），  β = β_prior + S⁻¹·γ。
 *       它保留了被测量真正确定的分量、把测量无法确定的分量按先验补上。
 *
 * @note **W 的选择很关键**：应尽量覆盖机械臂的运动空间（例如几百个随机位形
 *       + 随机速度/加速度），而不是只用某一条激励轨迹的回归矩阵。否则会把
 *       "这条轨迹上没激励到"误判成"结构性不可辨识"，搬回过多先验分量、
 *       抹掉本可辨识的信息。示例见 export_urdf_demo。
 *
 * @note 判秩用列归一化后的 W（与 kdl_lsq 一致，避免 kg 与 kg·m² 的量纲差异
 *       污染秩的判定），据此取出一组**零空间基** N；之后再在 N 张成的子空间里
 *       按 S 度量求解（z = (NᵀS²N)⁻¹NᵀS²(β_prior − β_id)）。
 *       这样"哪些方向可辨识"（= row(W)，一个固定子空间）与度量无关，
 *       度量只决定"在不可辨识的方向里怎么补"。反过来若直接按奇异值排序切
 *       子空间，排序会被缩放打乱、切出来的就不再是 row(W) —— 这是一个
 *       很容易写错、且结果看起来"差不多对"的陷阱。
 */
CompletionResult completeWithPrior(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & identified,
  const Eigen::VectorXd & prior, const CompletionOptions & options = CompletionOptions());

// ---------------------------------------------------------------------------
// 二、可表示性检查
// ---------------------------------------------------------------------------

/**
 * @brief 检查参数向量能否被 URDF 的 <inertial> 无损表示。
 * @param chain      [in] 运动学链（用于取段名做报错）。
 * @param parameters [in] 长度须 = 10 × 段数。
 * @param message    [out] 失败原因；成功时清空。
 * @return true 表示每一段都满足 "m > 0" 或 "m ≈ 0 且第一阶矩 ≈ 0"。
 *
 * @note 失败时 message 会指出是哪个段、以及为什么（便于判断是否需要补全）。
 */
bool parametersRepresentableInUrdf(
  const KDL::Chain & chain, const Eigen::VectorXd & parameters, std::string & message);

// ---------------------------------------------------------------------------
// 三、导出
// ---------------------------------------------------------------------------

/**
 * @brief 单个 link 的导出信息（用于打印/诊断）。
 */
struct UrdfLinkExport
{
  std::string link_name;                ///< URDF link 名（= 链段名）
  double mass = 0.0;                    ///< 写出去的质量 [kg]
  Eigen::Vector3d cog = Eigen::Vector3d::Zero();           ///< 质心（link 系）[m]
  Eigen::Matrix3d inertia_at_cog = Eigen::Matrix3d::Zero();  ///< 质心惯量 [kg·m²]
  bool replaced_existing = false;       ///< true = 覆盖了原有 <inertial>；false = 新增
};

/**
 * @brief URDF 导出的结果。
 */
struct UrdfExportResult
{
  bool success = false;                 ///< 是否成功
  std::string message;                  ///< 失败原因（中文、可直接打印）
  std::string output_file;              ///< 实际写出的文件路径
  std::vector<UrdfLinkExport> links;    ///< 被改写的 link（链顺序）
  unsigned int segments = 0;            ///< 链的段数
};

/**
 * @brief 用给定参数生成一份新的 URDF（只替换链上各段的 <inertial>）。
 * @param input_urdf  [in] 模板 URDF 路径（原文件不会被修改）。
 * @param chain       [in] 模板链：段名 → URDF link 名的映射来源。
 * @param parameters  [in] 长度须 = 10 × 段数，且须通过 parametersRepresentableInUrdf()。
 * @param output_urdf [in] 输出路径；父目录不存在会自动创建。
 * @return UrdfExportResult。
 *
 * @note 只改写"链上段"对应的 link；URDF 里其它 link（如 base_link、world）
 *       的 <inertial> 原样保留 —— 它们不在链上，不影响关节力矩。
 * @note 导出后**建议**调用 verifyExportedUrdf() 做一次往返校验。
 */
UrdfExportResult exportUrdfWithParameters(
  const std::string & input_urdf, const KDL::Chain & chain, const Eigen::VectorXd & parameters,
  const std::string & output_urdf);

// ---------------------------------------------------------------------------
// 四、往返校验
// ---------------------------------------------------------------------------

/**
 * @brief 往返校验的结果。
 */
struct UrdfRoundTripCheck
{
  bool success = false;                  ///< 重新解析 + 参数比对是否通过
  std::string message;                   ///< 失败原因
  double max_parameter_error = 0.0;      ///< max|β_重新解析 − β_期望|（绝对值）
  double max_relative_error = 0.0;       ///< 逐参数相对误差的最大值（分母取 max(|期望|,1)）
  unsigned int segments = 0;             ///< 重新解析出的段数
};

/**
 * @brief 重新解析导出的 URDF，检查参数是否与写出去的一致。
 * @param urdf_file           [in] 待校验的 URDF 路径。
 * @param expected_parameters [in] 期望的参数向量（即导出时写入的那份）。
 * @return UrdfRoundTripCheck。
 *
 * @note 这是"导出没写错"的**唯一可信证据**：它走的是 kdl_parser 的真实解析
 *       路径，而不是复用导出时的假设。示例里的判据取 1e-9。
 */
UrdfRoundTripCheck verifyExportedUrdf(
  const std::string & urdf_file, const Eigen::VectorXd & expected_parameters);

}  // namespace kdl_identification

#endif  // KDL_PARAMETER_IDENTIFICATION__KDL_URDF_EXPORT_HPP_
