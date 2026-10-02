// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务层打印工具的实现。

#include "router/kdl_control_print.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>

#include "kdl_interpolation_print.hpp"

namespace kdl_control
{
namespace
{

/// 段数超过这个值就不再打印路点表：笛卡尔任务重建出来的轨迹可能上千段，
/// 全打出来只会把终端刷满（要看细节请自己调 printKnotTable）。
constexpr unsigned int kMaxKnotPrint = 12;

/// 打印"标签 + 关节向量"（printJointVector 自带换行，标签由本函数补）。
void printLabeledVector(const char * label, const KDL::JntArray & values, std::ostream & os)
{
  os << "  " << label;
  kdl_interpolation::printJointVector(values, os);
}

/// 打印一项笛卡尔标量约束：<= 0 表示调用者没设，如实写"不校验"。
void printCartesianBound(const char * label, const char * unit, double value, std::ostream & os)
{
  os << "  " << label;
  if (value > 0.0) {
    os << value << " " << unit << "\n";
  } else {
    os << "不校验\n";
  }
}

}  // namespace

void printTaskStatus(TaskStatus status, std::ostream & os)
{
  os << "  任务状态: " << taskStatusName(status) << "\n";
}

void printRobotContext(const RobotContext & ctx, std::ostream & os)
{
  const unsigned int n = ctx.chain.getNrOfJoints();
  os << "[RobotContext] " << n << " 个可动关节, " << ctx.chain.getNrOfSegments() << " 段"
     << (ctx.valid() ? "（装配完整）\n" : "（装配不完整！）\n");
  if (n == 0) {
    return;
  }

  printLabeledVector("关节下限 (rad): ", ctx.q_min, os);
  printLabeledVector("关节上限 (rad): ", ctx.q_max, os);
  if (ctx.max_torque.rows() == n) {
    printLabeledVector("力矩上限 (N·m): ", ctx.max_torque, os);
  } else {
    os << "  力矩上限: 未设置（不校验力矩）\n";
  }

  os << "  关节约束（长度 0 / <= 0 表示不校验）:\n";
  kdl_interpolation::printJointLimits(ctx.joint_limits, os);

  os << "  笛卡尔约束:\n";
  printCartesianBound("|v| 上限   : ", "m/s", ctx.cartesian_limits.max_linear_velocity, os);
  printCartesianBound(
    "|a| 上限   : ", "m/s²", ctx.cartesian_limits.max_linear_acceleration, os);
  printCartesianBound("|ω| 上限   : ", "rad/s", ctx.cartesian_limits.max_angular_velocity, os);
  printCartesianBound(
    "|α| 上限   : ", "rad/s²", ctx.cartesian_limits.max_angular_acceleration, os);
}

void printControlResult(const ControlResult & result, std::ostream & os)
{
  os << "[ControlResult] 任务 = " << taskTypeName(result.type) << "\n";
  os << "  结论: " << (result.success ? "成功" : "失败") << "\n";
  if (!result.message.empty()) {
    os << "  原因: " << result.message << "\n";
  }

  if (result.joint_trajectory.valid()) {
    if (result.joint_trajectory.segmentCount() <= kMaxKnotPrint) {
      kdl_interpolation::printTrajectorySummary(result.joint_trajectory, os);
      kdl_interpolation::printKnotTable(result.joint_trajectory, os);
    } else {
      os << "  [QuinticTrajectory] " << result.joint_trajectory.joints() << " 关节, "
         << result.joint_trajectory.segmentCount() << " 段, 总时长 "
         << result.joint_trajectory.duration()
         << " s（段数较多，略去路点表；细节请调 kdl_interpolation::printKnotTable()）\n";
    }
  } else {
    os << "  轨迹: 无（未生成）\n";
  }

  if (result.type == TaskType::kCartesianSpace) {
    // 只有在真的重建出关节轨迹、跑过闭环自检之后，这几个数字才有意义；
    // 更早失败（IK 未收敛、重建不可行）时打印 0 会误导读者。
    if (result.joint_trajectory.valid()) {
      os << "  闭环自检: 位置残差峰值 " << result.position_error << " m, 姿态残差峰值 "
         << result.orientation_error << " rad, IK 失败 " << result.ik_failures
         << " 点, 路径最小奇异值 " << result.min_singular_value << "\n";
    } else {
      os << "  闭环自检: 未执行（尚未生成关节轨迹；IK 失败 " << result.ik_failures << " 点）\n";
    }
  }

  if (!result.torque_feedforward.empty()) {
    os << "  力矩前馈: 已算 " << result.torque_feedforward.size()
       << " 个采样点（峰值见 printTorqueFeedforward）\n";
  }
}

void printTorqueFeedforward(const ControlResult & result, std::ostream & os)
{
  if (result.torque_feedforward.empty()) {
    os << "[TorqueFeedforward] 无数据（未开启 compute_torque_feedforward？）\n";
    return;
  }

  const auto & samples = result.torque_feedforward;
  const auto & times = result.torque_times;
  const unsigned int joints = samples.front().rows();

  os << "[TorqueFeedforward] " << samples.size() << " 个采样点, " << joints << " 个关节\n";
  for (unsigned int j = 0; j < joints; ++j) {
    double peak = 0.0;
    double peak_time = 0.0;
    for (std::size_t k = 0; k < samples.size(); ++k) {
      const double torque = samples[k](j);
      if (std::abs(torque) > peak) {
        peak = std::abs(torque);
        peak_time = (k < times.size()) ? times[k] : 0.0;
      }
    }
    os << "  关节 " << (j + 1) << ": 峰值 |τ| = " << peak << " N·m（t = " << peak_time
       << " s）\n";
  }
}

}  // namespace kdl_control
