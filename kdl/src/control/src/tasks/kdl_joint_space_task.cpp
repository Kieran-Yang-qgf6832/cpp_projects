// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务1（关节空间运动）的实现。

#include "tasks/kdl_joint_space_task.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace kdl_control
{
namespace
{

/// 单段峰值速度系数：|q̇|max = 15·Δq/(8T) = 1.875·Δq/T，见 kdl_quintic.hpp 文件头。
constexpr double kPeakVelocityFactor = 15.0 / 8.0;

/**
 * @brief 自动定时的安全余量：算出来的 T 再乘 1.02。
 *
 * @note 反推用的是**解析峰值**，而 kdl_interpolation 的速度/加速度校验是**密集采样**
 *       取峰值，两者在末位上总会差最后一位。正好卡在上限上时，会出现
 *       "峰值 0.500000 超过上限 0.500000" 这种一眼看去荒唐、但完全符合代码逻辑的报错。
 *       留 2% 余量比事后去解释这个边界现象要省事，也符合"别贴着约束边界干活"的工程习惯。
 */
constexpr double kDurationSafetyFactor = 1.02;

/// 把数值格式化成可读字符串（只用于拼错误信息，故各 .cpp 各自持有这一份小工具）。
std::string num(double value, int precision = 6)
{
  std::ostringstream os;
  os << std::setprecision(precision) << value;
  return os.str();
}

}  // namespace

bool JointSpaceTask::estimateDuration(
  const RobotContext & ctx, const KDL::JntArray & q_from, const KDL::JntArray & q_to,
  double & duration, std::string & message)
{
  const unsigned int n = ctx.chain.getNrOfJoints();
  if (q_from.rows() != n || q_to.rows() != n) {
    message = "自动定时失败：起点/终点关节角长度与链的关节数不一致";
    return false;
  }
  if (!ctx.joint_limits.check_velocity()) {
    message = "自动定时失败：未设置关节速度上限（joint_limits.max_velocity），"
              "既不知道该多快，就无法反推时长；请显式给 duration，或先设置速度上限";
    return false;
  }
  if (ctx.joint_limits.max_velocity.rows() != n) {
    message = "自动定时失败：速度上限长度 " +
              std::to_string(ctx.joint_limits.max_velocity.rows()) + " 与关节数 " +
              std::to_string(n) + " 不一致";
    return false;
  }

  double longest = 0.0;
  bool any_bounded = false;
  for (unsigned int i = 0; i < n; ++i) {
    const double v_max = ctx.joint_limits.max_velocity(i);
    if (v_max <= 0.0) {
      continue;  // 该关节没设速度上限，不参与定时
    }
    any_bounded = true;
    // T ≥ 1.875·|Δq| / v_max：逐关节算所需的最短时长，取最大者（以最快关节为准）
    const double needed = kPeakVelocityFactor * std::abs(q_to(i) - q_from(i)) / v_max;
    longest = std::max(longest, needed);
  }

  if (!any_bounded) {
    message = "自动定时失败：速度上限全为 0 或负值（相当于没有约束），无法反推时长";
    return false;
  }

  if (longest <= 0.0) {
    // 位移全为 0：目标已达成。给一个最短保持段，而不是生成 T = 0 的非法段 ——
    // 上层仍会得到一条（静止的）合法轨迹，可以照常下发给控制器。
    duration = kMinSegementDuration;
    return true;
  }

  duration = longest * kDurationSafetyFactor;
  return true;
}

bool JointSpaceTask::validate(
  const RobotContext & ctx, const TaskRequest & req, std::string & message) const
{
  const unsigned int n = ctx.chain.getNrOfJoints();

  if (req.goal_joint.rows() != n) {
    message = "目标任务关节角长度 " + std::to_string(req.goal_joint.rows()) +
              " 与链的关节数 " + std::to_string(n) + " 不匹配";
    return false;
  }

  // 目标必须落在行程内。限位来自 URDF，控制层不做 clamp：越界是调用者的意图问题，
  // 悄悄夹到边界上会让"实际去的地方"和"要求去的地方"不一致。
  if (ctx.q_min.rows() == n && ctx.q_max.rows() == n) {
    for (unsigned int i = 0; i < n; ++i) {
      if (req.goal_joint(i) < ctx.q_min(i) || req.goal_joint(i) > ctx.q_max(i)) {
        message = "第 " + std::to_string(i + 1) + " 个关节的目标 " + num(req.goal_joint(i)) +
                  " rad 超出行程 [" + num(ctx.q_min(i)) + ", " + num(ctx.q_max(i)) + "] rad";
        return false;
      }
    }
  }

  if (req.durations.size() > 1) {
    message = "关节空间任务目前只支持单段：durations 长度应为 1（收到 " +
              std::to_string(req.durations.size()) + "）";
    return false;
  }
  if (!req.durations.empty() && req.durations.front() <= 0.0) {
    message = "durations[0] 必须 > 0（收到 " + num(req.durations.front()) + " s）";
    return false;
  }
  if (req.duration < 0.0) {
    message = "duration 必须 >= 0（0 表示自动估算，收到 " + num(req.duration) + " s）";
    return false;
  }

  return true;
}

ControlResult JointSpaceTask::solve(
  const RobotContext & ctx, const TaskRequest & req, const KDL::JntArray & q_now,
  const KDL::JntArray & qdot_now) const
{
  // 本模块生成的轨迹两端静止（kStop 语义），当前速度不参与解算，接口上保留它
  // 只是为了将来做"起步速度衔接"时有位置可放。
  (void)qdot_now;

  ControlResult result;
  result.type = TaskType::kJointSpace;

  // ---- 1. 段时长：调用者给的优先，否则按速度上限反推 ----
  double duration = 0.0;
  if (!req.durations.empty()) {
    duration = req.durations.front();
  } else if (req.duration > 0.0) {
    duration = req.duration;
  } else if (!estimateDuration(ctx, q_now, req.goal_joint, duration, result.message)) {
    return result;
  }

  // ---- 2. 组两个路点，剩下的交给插值模块 ----
  // 它内部会做整条轨迹的速度/加速度/jerk 校验，所以这里不需要（也不应该）
  // 再实现一遍限位判定，只把它的结论原样透传。
  const std::vector<KDL::JntArray> waypoints{ q_now, req.goal_joint };
  const std::vector<double> durations{ duration };

  const kdl_interpolation::TrajectoryResult build =
    kdl_interpolation::buildQuinticTrajectory(waypoints, durations, ctx.joint_limits);
  if (!build.success) {
    result.message = "构建关节空间轨迹失败：" + build.message;
    return result;
  }

  // ---- 3. 收尾 ----
  result.success = true;
  result.joint_trajectory = build.trajectory;
  result.duration = result.joint_trajectory.duration();
  return result;
}

}  // namespace kdl_control
