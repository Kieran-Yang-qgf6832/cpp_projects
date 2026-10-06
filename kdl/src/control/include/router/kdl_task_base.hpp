// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务层（control）的公共数据结构与任务基类。
//
// ===========================================================================
// 这一层在整条链路里的位置
// ===========================================================================
//   Tools          URDF -> KDL::Tree / KDL::Chain
//   Kinematics     正运动学 / 雅可比 / 逆运动学
//   Interpolation  关节空间与笛卡尔空间插值（只吃"路点 + 段时长"）
//   Dynamics       逆动力学（给定运动求所需力矩）
//   control        ← 本模块：把"任务"翻译成对上面四层的调用序列
//
// 三条边界（后面所有取舍都能回溯到它们）：
//   1) 只调用、不修改底层模块：control 的改动永远不会波及 KDL 与已有模块；
//   2) **所有任务的结果都落到同一种东西——关节空间轨迹 QuinticTrajectory**：
//      上层 ros2_control 只能下发关节量，笛卡尔任务也必须经 IK 落回关节侧，
//      于是调用方只需要认识一种结果；
//   3) 失败用"结果结构 + 中文 message"表达，不抛异常：调用方可能正处在实时
//      控制循环里，异常会让整条链路失去可预期的行为。
// ===========================================================================

#ifndef KDL_CONTROL__KDL_TASK_BASE_HPP_
#define KDL_CONTROL__KDL_TASK_BASE_HPP_

#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_cartesian.hpp"
#include "kdl_quintic.hpp"

namespace kdl_control
{

// ---------------------------------------------------------------------------
// 一、任务的类型与状态
// ---------------------------------------------------------------------------

/**
 * @brief 任务类型：路由按这个字段分派到具体任务实现。
 *
 * @note 新增任务时：① 这里加枚举值；② 写一个 TaskBase 派生类；
 *       ③ 在 TaskRouter 里 registerTask()。其余代码不用动。
 */
enum class TaskType
{
  kUnknown,        ///< 未识别 / 尚未实现
  kJointSpace,     ///< 任务1：关节空间运动（当前关节状态 → 目标关节状态）
  kCartesianSpace  ///< 任务2：笛卡尔空间运动（当前末端位姿 → 目标末端位姿）
};

/**
 * @brief 任务状态。
 *
 * @note 解算本身是纯函数（不保存状态），这个枚举是给上层状态机用的：
 *       例如"下发前校验通过 → kReady；轨迹执行到总时长 → kFinished"。
 */
enum class TaskStatus
{
  kIdle,      ///< 未开始
  kReady,     ///< 校验通过，等待下发
  kRunning,   ///< 执行中
  kFinished,  ///< 到位
  kFailed     ///< 失败（message 说明原因）
};

/**
 * @brief 任务类型对应的可读名字。
 * @param type [in] 任务类型。
 * @return 常量字符串，未实现时返回 "unimplemented"。
 */
const char * taskTypeName(TaskType type);

/**
 * @brief 任务状态对应的可读名字。
 * @param status [in] 任务状态。
 * @return 常量字符串。
 */
const char * taskStatusName(TaskStatus status);

// ---------------------------------------------------------------------------
// 二、机器人上下文：任务的"静态前提"
// ---------------------------------------------------------------------------

/**
 * @brief 解算任务所需要的机器人静态信息：装配一次，所有任务复用。
 *
 * @note 这里放的都是"不随时间变化"的东西（链、行程、力矩上限、各项速度上限），
 *       而"当前关节状态"必须每次解算时由调用者传进来 —— 把可变状态留在任务里
 *       是最容易出"用了上一周期的 q"这类隐蔽 bug 的地方。
 * @note 各限位数组的"长度 0 = 不校验"语义与 kdl_interpolation 保持一致：
 *       max_torque 为空时只算力矩前馈、不做上限判定。
 */
struct RobotContext
{
  /// 由 URDF 截取出来的运动学链（kdl_tools::buildChainFromUrdfFile 的产物）。
  KDL::Chain chain{};

  /// 关节位置下限 [rad]（平移关节为 m），长度 = 可动关节数。
  KDL::JntArray q_min{};

  /// 关节位置上限 [rad]（平移关节为 m），长度 = 可动关节数。
  KDL::JntArray q_max{};

  /// 关节力矩上限 [N·m]（来自 URDF <limit effort>）；长度 0 表示不校验。
  KDL::JntArray max_torque{};

  /// 关节速度/加速度/jerk 上限，交给 kdl_interpolation 判定轨迹可行性。
  kdl_interpolation::JointLimits joint_limits{};

  /// 末端线/角速度、线/角加速度上限，交给 kdl_interpolation 判定笛卡尔可行性。
  kdl_interpolation::CartesianLimits cartesian_limits{};

  /**
   * @brief 上下文是否装配完整。
   * @return true 表示链里有可动关节，且行程数组长度与关节数一致、区间合法，
   *         且 max_torque 或为空（不校验）或长度一致。
   */
  bool valid() const;
};

// ---------------------------------------------------------------------------
// 三、任务信号与结果
// ---------------------------------------------------------------------------

/**
 * @brief 任务信号：调用者填这个结构体，路由据此选任务并解算。
 *
 * @note 时间参数的优先级（两条任务一致）：
 *       durations 非空 → 逐段使用；否则 duration > 0 → 用它；否则按限位自动估算。
 *       关节空间任务目前只支持单段，所以它的 durations 长度必须为 1。
 */
struct TaskRequest
{
  /// 任务类型：路由的分派依据。
  TaskType type = TaskType::kUnknown;

  /// 可选的任务名，只用于日志/打印。
  std::string name{};

  // ---- 任务1（关节空间）用 ----
  /// 目标关节角 [rad]，长度必须 = 关节数。
  KDL::JntArray goal_joint{};

  // ---- 任务2（笛卡尔空间）用 ----
  /// 单段目标位姿（相对基座）；cartesian_waypoints 非空时忽略本字段。
  KDL::Frame goal_pose{};

  /**
   * @brief 中间路点 + 目标位姿（相对基座）。
   *
   * @note **起点由任务自己插入**：实际路径 = {起点位姿} + cartesian_waypoints。
   *       所以本数组里只需要给"中途要经过的点"和"最后一站"，不必重复写起点。
   *       数组为空时就退化成单段：{起点, goal_pose}。
   * @note 单段的语义固定为"位置走直线 + 姿态绕固定轴转最短弧"；要弧线、绕行，
   *       或希望末端"不沿直线运动"，就给中间路点（段数 = 本数组长度）。
   */
  std::vector<KDL::Frame> cartesian_waypoints{};

  /// 是否显式指定起点位姿；为 false 时用 FK(q_now) 作为起点（保证与实际状态一致）。
  bool has_start_pose = false;

  /// 显式起点位姿（has_start_pose 为 true 时有效）。
  KDL::Frame start_pose{};

  // ---- 时间参数（两条任务共用） ----
  /// 单段时长 [s]；<= 0 表示按限位自动估算。多段时表示"每段都用这个时长"。
  double duration = 0.0;

  /// 逐段时长 [s]，长度必须 = 段数；非空时优先级高于 duration。
  std::vector<double> durations{};

  /// 内部路点处的行为（只影响位置，姿态两种模式都会停住）；仅任务2有效。
  kdl_interpolation::WaypointBehavior behavior = kdl_interpolation::WaypointBehavior::kStop;

  // ---- 开关与阈值 ----
  /// 是否调用 Dynamics 计算力矩前馈（τ = M·q̈ + C·q̇ + G）。
  bool compute_torque_feedforward = false;

  /// 是否用 RobotContext::max_torque 判定轨迹的力矩可行性（超限即判失败）。
  bool check_torque_limit = false;

  /// 采样步长 [s]：任务2 用它把笛卡尔轨迹离散成 IK 点，也用于力矩前馈采样。
  double sample_dt = 0.01;

  /**
   * @brief 任务2 重建关节轨迹时的路点间距 [s]（必须 > 0）。
   *
   * @note 它和 sample_dt 分工不同：sample_dt 决定"多密地采样参考路径并逆解"
   *       （越密，路径保真度自检越严格），rebuild_dt 决定"用多稀的路点把关节角
   *       串成 C⁴ 轨迹"。
   * @note 为什么重建必须比采样稀得多：C⁴ 连续条件是由路点位置的高阶差分决定的
   *       （jerk ~ 1/T³、snap ~ 1/T⁴），而逆解给出的关节角带 ~1e-6 rad 量级的噪声。
   *       用"已知平滑轨迹 + 高斯噪声"实测到的 jerk 峰值（rad/s³，真值 1.33）：
   *
   *         路点间距     噪声 1e-6     噪声 1e-5     噪声 1e-4
   *          10 ms          76           1064          4526
   *          25 ms           4.7           59           640
   *          50 ms           1.7            7.0          66
   *         100 ms           1.4            2.2          10
   *         200 ms           1.34 (≈真值)   1.4           2.1
   *
   *       噪声按 ≈1/T³ 放大：10 ms 的路点间距会把 1e-6 rad 的噪声放大成 76 rad/s³
   *       的假 jerk，把 jerk 上限判成"必然超限"。默认取 100 ms（对上表里 1e-5 噪声
   *       只放大到 2.2），既让速度/加速度/jerk 重新成为有意义的物理量，
   *       又保留足够多的路点保证路径保真度（自检会盯着这件事）。
   */
  double rebuild_dt = 0.1;

  /// 任务2 的 IK 解跳变阈值 [rad]：相邻采样点关节角差超过它，判为翻关节/绕奇异。
  double joint_jump_threshold = 0.5;

  /// 任务2 的自检位置残差阈值 [m]。
  double position_tolerance = 1e-3;

  /// 任务2 的自检姿态残差阈值 [rad]（默认约 0.5°）。
  double orientation_tolerance = 8.7e-3;
};

/**
 * @brief 解算结果：所有任务同构，上层只需认识这一种结果。
 */
struct ControlResult
{
  /// 是否成功。
  bool success = false;

  /// 失败原因（中文、可直接打印）；成功时为空。
  std::string message{};

  /// 本结果来自哪一类任务。
  TaskType type = TaskType::kUnknown;

  /**
   * @brief 解算出来的关节空间轨迹（统一出口）。
   *
   * @note 成功时一定有效。**因"力矩超限"或"FK 自检超差"而失败时它仍然保留**，
   *       因为这两类失败下轨迹本身是完整的，调用方可能想降速重试或直接分析，
   *       丢掉它反而让排查更难。
   */
  kdl_interpolation::QuinticTrajectory joint_trajectory{};

  /// 轨迹总时长 [s]（= joint_trajectory.duration()）。
  double duration = 0.0;

  /// 力矩前馈序列（compute_torque_feedforward 为真且 Dynamics 可用时才有内容）。
  std::vector<KDL::JntArray> torque_feedforward{};

  /// 与 torque_feedforward 一一对应的时刻 [s]。
  std::vector<double> torque_times{};

  /// 任务2 的 FK 自检位置残差峰值 [m]；任务1 恒为 0。
  double position_error = 0.0;

  /**
   * @brief 任务2 的 FK 自检姿态残差峰值 [rad]；任务1 恒为 0。
   *
   * @note **含义随逆解模式变**，因为 4 轴臂的姿态任务空间只有一维：
   *       - 解析解模式（链是"平面 2R + 2 自由度腕部"）：只统计**绕工具轴滚转**的
   *         残差，也就是这个臂真正能控制的那一维；
   *       - 数值解模式（其它链）：统计完整姿态夹角。
   *       两种模式下它都是"自检通过与否"的判据之一（阈值 orientation_tolerance）。
   */
  double orientation_error = 0.0;

  /**
   * @brief 参考姿态与实到姿态的**完整**夹角峰值 [rad]（仅解析解模式非 0）。
   *
   * @note 只做诊断，**不参与通过与否的判定**。4 轴臂够不着参考姿态里"工具轴指向"
   *       那一维，所以这个数天然不会小：它回答的是"我们要求的姿态离这个臂真正能到的
   *       姿态流形有多远"。它大不代表解算错了，只代表目标姿态本身超出了构型能力；
   *       真要小只能改目标。
   */
  double pose_residual = 0.0;

  /// 任务2 中 IK 未收敛的采样点数（成功时为 0）。
  unsigned int ik_failures = 0;

  /**
   * @brief 任务2 路径上雅可比最小奇异值的最小值（离奇异面有多远）；任务1 恒为 0。
   *
   * @note 本机器人（4 轴臂）上的参照量级：零位（直臂，奇异）≈ 0.0185，
   *       常用构型 ≈ 0.18~0.21。低于 0.01 会在解算时直接判失败（见 CartesianSpaceTask
   *       的阈值说明），所以成功时这个数一般大于 0.01 —— 但越大越好，越小说明越贴
   *       奇异面。注意 0.0185 / 0.01 只有约 1.85 倍余量，换模型后应当重新量。
   */
  double min_singular_value = 0.0;
};

// ---------------------------------------------------------------------------
// 四、任务基类
// ---------------------------------------------------------------------------

/**
 * @brief 任务基类：所有任务实现（tasks/ 下）都从这里派生。
 *
 * @note solve() 被设计成**纯函数**：任务对象不持有"当前关节状态"，状态由调用者
 *       每个周期传进来。这样同一个任务实例既能被反复调用，也方便在示例里对同
 *       一组输入比较不同实现的结果。
 */
class TaskBase
{
public:
  virtual ~TaskBase() = default;

  /// @return 任务类型（路由的键）。
  virtual TaskType type() const = 0;

  /// @return 任务名，用于打印/日志。
  virtual const char * name() const = 0;

  /**
   * @brief 前置校验：尺寸、限位、参数自洽性。不计算轨迹。
   * @param ctx     [in]  机器人上下文。
   * @param req     [in]  任务信号。
   * @param message [out] 失败原因（中文）。
   * @return true 表示可以进入 solve()。
   *
   * @note 拆成独立函数是为了让上层"下发前先廉价校验一次"，校验不过就不必做
   *       后面昂贵的 IK/插值计算。
   */
  virtual bool validate(
    const RobotContext & ctx, const TaskRequest & req, std::string & message) const = 0;

  /**
   * @brief 解算：由当前状态 + 任务请求生成关节空间轨迹。
   * @param ctx       [in] 机器人上下文。
   * @param req       [in] 任务信号。
   * @param q_now     [in] 当前关节角 [rad]，长度 = 关节数。
   * @param qdot_now  [in] 当前关节速度 [rad/s]，长度 = 关节数（或为 0 长度表示未知）。
   * @return 解算结果；失败时 message 说明原因。
   *
   * @note 本层生成的轨迹一律"两端静止"（kdl_interpolation 的 kStop 语义），
   *       所以 qdot_now 不参与解算，保留它只是为了接口完整、并为将来做
   *       "起步速度衔接"留位置。
   */
  virtual ControlResult solve(
    const RobotContext & ctx, const TaskRequest & req, const KDL::JntArray & q_now,
    const KDL::JntArray & qdot_now) const = 0;
};

}  // namespace kdl_control

#endif  // KDL_CONTROL__KDL_TASK_BASE_HPP_
