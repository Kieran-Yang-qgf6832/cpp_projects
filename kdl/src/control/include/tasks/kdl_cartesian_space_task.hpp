// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务2——笛卡尔空间运动（当前末端位姿 → 目标末端位姿）。
//
// ===========================================================================
// 难点：笛卡尔轨迹怎么变成"能下发"的关节轨迹
// ===========================================================================
// 上层（ros2_control）只认识关节量，所以笛卡尔任务必须落回关节侧。做法是
// "离散 + 逐点 IK + 关节侧重建"：
//
//   ① 用 kdl_interpolation 造笛卡尔轨迹（位置五次 + 姿态 slerp）；
//   ② 按 sample_dt 把它离散成一串参考位姿；
//   ③ 逐点逆解（**热启动**：以上一点的解作为本次初值），得到关节角序列；
//   ④ 把关节角序列当路点，再用 buildQuinticTrajectory() 重建一条关节轨迹。
//
// ---------------------------------------------------------------------------
// 为什么不用"雅可比速度映射"直接拼关节轨迹
// ---------------------------------------------------------------------------
// 也可以走 ② → q̇ = J⁻¹·V、q̈ = J⁻¹·(A − J̇q̇) 这条路，但有两个硬问题：
//   1) QuinticTrajectory 只能由 buildQuinticTrajectory() 产生（构造函数私有、
//      入参就是"路点 + 段时长"）。要把任意 (q, q̇, q̈) 塞进去就必须改插值模块，
//      违反"control 只调用、不修改底层"的边界；
//   2) 反过来，采样足够密时上面 ④ 步的插值误差是 O(sample_dt⁴)（五次多项式），
//      而且这个误差**可以被量出来**（见下），比"雅可比算出来的更准"这种没有
//      依据的说法可靠得多。
//
// ---------------------------------------------------------------------------
// 误差是量出来的，不是猜的
// ---------------------------------------------------------------------------
// 重建出关节轨迹后，本任务会做一次闭环自检：在同样的时刻对重建轨迹采样 →
// 正向运动学 → 与笛卡尔参考位姿比较，取位置/姿态残差峰值填进结果。
// 残差超阈值就先减半 sample_dt 重算一次，仍超差才判失败并建议增加中间路点。
//
// 代价要说清楚：重建后关节轨迹的**速度**由五次插值重新解出，与"把笛卡尔速度
// 经 J⁻¹ 映射到关节侧"得到的值有 O(sample_dt²) 差异。需要更贴近就减小 sample_dt
// （默认 0.01 s，正好对齐 ros2_control 的 100 Hz 控制周期）。
//
// ---------------------------------------------------------------------------
// "不一定沿直线运动"
// ---------------------------------------------------------------------------
// 单段的语义是固定的：位置走直线（三个分量共用同一个 s(τ) 形状）、姿态绕固定轴
// 转最短弧。要弧线、绕行、或者只是不想走直线，就给中间路点
// （TaskRequest::cartesian_waypoints），段数 = 路点数，起点由本任务自己插入。
// 内部路点处的行为由 WaypointBehavior 选：
//   kStop        每个路点精确停住（位置与姿态行为一致），默认、最安全；
//   kPassThrough 位置路过内部路点不减速，但姿态仍会停住 —— 这是 slerp 段
//                "绕固定轴转"带来的硬约束，不是实现偷懒（详见 kdl_cartesian.hpp）。
// ===========================================================================

#ifndef KDL_CONTROL__KDL_CARTESIAN_SPACE_TASK_HPP_
#define KDL_CONTROL__KDL_CARTESIAN_SPACE_TASK_HPP_

#include <string>

#include "router/kdl_task_base.hpp"

namespace kdl_control
{

/**
 * @brief 任务2：笛卡尔空间运动。
 */
class CartesianSpaceTask : public TaskBase
{
public:
  CartesianSpaceTask() = default;
  ~CartesianSpaceTask() override = default;

  /// @return TaskType::kCartesianSpace。
  TaskType type() const override { return TaskType::kCartesianSpace; }

  /// @return "cartesian_space"。
  const char * name() const override { return "cartesian_space"; }

  bool validate(
    const RobotContext & ctx, const TaskRequest & req, std::string & message) const override;

  ControlResult solve(
    const RobotContext & ctx, const TaskRequest & req, const KDL::JntArray & q_now,
    const KDL::JntArray & qdot_now) const override;

  /**
   * @brief 按末端限位反推单段时长。
   * @param ctx      [in]  机器人上下文（需已设置 cartesian_limits）。
   * @param from     [in]  起点位姿。
   * @param to       [in]  终点位姿。
   * @param duration [out] 反推得到的时长 [s]，取速度与加速度两条下界的较大者。
   * @param message  [out] 失败原因（中文）。
   * @return true 表示成功。
   *
   * @note 峰值系数同样取自 kdl_cartesian.hpp 文件头：
   *       速度侧 |v|max = 15Δ/(8T)、|ω|max = 15θ/(8T)；
   *       加速度侧 |a|max = 10Δ/(√3 T²)、|α|max = 10θ/(√3 T²)。
   *       把加速度也算进去，是为了减少"自动定时后又被限位判为不可行"的概率。
   * @note 该公式对 kStop 模式的每一段都严格成立；kPassThrough 模式下内部路点
   *       速度不为 0，本估算只是**下界**（实际需要更长），这一点在结果里会被
   *       插值模块的可行性校验兜住。
   */
  static bool estimateDuration(
    const RobotContext & ctx, const KDL::Frame & from, const KDL::Frame & to, double & duration,
    std::string & message);

  /// 采样点数的上限（防止 duration 很小 / sample_dt 很小时把内存撑爆）。
  static constexpr unsigned int kMaxSamples = 5000;
};

}  // namespace kdl_control

#endif  // KDL_CONTROL__KDL_CARTESIAN_SPACE_TASK_HPP_
