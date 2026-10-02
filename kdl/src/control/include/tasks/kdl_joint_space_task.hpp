// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务1——关节空间运动（当前关节状态 → 目标关节状态）。
//
// ===========================================================================
// 这个任务在做什么
// ===========================================================================
// 输入：当前关节角 q_now、目标关节角 q_goal、以及时间参数；
// 输出：一条把 q_now 送到 q_goal 的关节空间轨迹（两端静止）。
//
// 全部算法都在 kdl_interpolation 里，本任务只做三件事：
//   1) 校验目标（长度、是否在行程内）；
//   2) 决定段时长 T（调用者给，或者按速度上限反推）；
//   3) 组两个路点交给 buildQuinticTrajectory()，并把它内部的限位校验结论
//      原样透传给调用者。
//
// ===========================================================================
// 没有给时长时，T 是怎么算出来的
// ===========================================================================
// 用的是 kdl_quintic.hpp 文件头给出的解析峰值。单段、两端 v = a = 0 时
//
//     |q̇|max = 15·Δq / (8T) = 1.875·Δq / T        （峰值出现在 τ = 0.5）
//
// 想让它不超过关节速度上限 v_max，就得让
//
//     T ≥ 1.875·|Δq| / v_max
//
// 逐关节各算一遍取最大值，就是"以最快关节为准"的最短可行时长。这不是拍脑袋
// 的系数：它和插值模块用的是同一条结论，所以自动定时与随后的可行性校验不会
// 自相矛盾（真要矛盾，也只可能是加速度/jerk 上限更紧，那时 buildQuinticTrajectory
// 会如实报超限，由调用者放慢）。
// ===========================================================================

#ifndef KDL_CONTROL__KDL_JOINT_SPACE_TASK_HPP_
#define KDL_CONTROL__KDL_JOINT_SPACE_TASK_HPP_

#include <string>

#include "router/kdl_task_base.hpp"

namespace kdl_control
{

/**
 * @brief 任务1：关节空间运动。
 *
 * @note 目前只支持**单段**（q_now → q_goal 一条五次多项式），多段关节路点属于
 *       后续扩展（接口上是 TaskRequest::durations 长度大于 1，本实现会明确报错
 *       而不是悄悄只用第一段）。
 */
class JointSpaceTask : public TaskBase
{
public:
  JointSpaceTask() = default;
  ~JointSpaceTask() override = default;

  /// @return TaskType::kJointSpace。
  TaskType type() const override { return TaskType::kJointSpace; }

  /// @return "joint_space"。
  const char * name() const override { return "joint_space"; }

  bool validate(
    const RobotContext & ctx, const TaskRequest & req, std::string & message) const override;

  ControlResult solve(
    const RobotContext & ctx, const TaskRequest & req, const KDL::JntArray & q_now,
    const KDL::JntArray & qdot_now) const override;

  /**
   * @brief 按关节速度上限反推单段时长 T = max_i(1.875·|Δq_i| / v_max_i)。
   * @param ctx      [in]  机器人上下文（需已设置 joint_limits.max_velocity）。
   * @param q_from   [in]  起点关节角。
   * @param q_to     [in]  终点关节角。
   * @param duration [out] 反推得到的时长 [s]。
   * @param message  [out] 失败原因（中文）。
   * @return true 表示成功；false 表示未设速度上限、长度不匹配或位移过小。
   *
   * @note 公开出来是为了让示例/上层能"看看这个 T 是怎么来的"：示例会把逐关节的
   *       1.875·|Δq_i| / v_max_i 与最终取到的 T 并排打印，结论肉眼可验证。
   */
  static bool estimateDuration(
    const RobotContext & ctx, const KDL::JntArray & q_from, const KDL::JntArray & q_to,
    double & duration, std::string & message);

  /// 位移过小（目标已达成）时使用的最短段时长 [s]，避免生成非法（T <= 0）的段。
  static constexpr double kMinSegementDuration = 0.1;
};

}  // namespace kdl_control

#endif  // KDL_CONTROL__KDL_JOINT_SPACE_TASK_HPP_
