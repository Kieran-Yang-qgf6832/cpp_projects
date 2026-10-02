// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务层公共数据结构的实现（名字映射与上下文自检）。

#include "router/kdl_task_base.hpp"

namespace kdl_control
{

const char * taskTypeName(TaskType type)
{
  switch (type) {
    case TaskType::kJointSpace:
      return "joint_space";
    case TaskType::kCartesianSpace:
      return "cartesian_space";
    case TaskType::kUnknown:
    default:
      return "unimplemented";
  }
}

const char * taskStatusName(TaskStatus status)
{
  switch (status) {
    case TaskStatus::kIdle:
      return "idle";
    case TaskStatus::kReady:
      return "ready";
    case TaskStatus::kRunning:
      return "running";
    case TaskStatus::kFinished:
      return "finished";
    case TaskStatus::kFailed:
    default:
      return "failed";
  }
}

bool RobotContext::valid() const
{
  const unsigned int n = chain.getNrOfJoints();
  if (n == 0) {
    return false;
  }

  // 行程必须逐关节齐全：q_min/q_max 少一个，后面的越界判定与自动定时都会失真，
  // 所以宁可判"上下文没装好"，也不要带着半份数据继续算。
  if (q_min.rows() != n || q_max.rows() != n) {
    return false;
  }
  for (unsigned int i = 0; i < n; ++i) {
    if (q_min(i) > q_max(i)) {
      return false;
    }
  }

  // 力矩上限是**可选**的：长度 0 表示"不校验力矩"（与 JointLimits 的约定一致）。
  if (max_torque.rows() != 0 && max_torque.rows() != n) {
    return false;
  }

  return true;
}

}  // namespace kdl_control
