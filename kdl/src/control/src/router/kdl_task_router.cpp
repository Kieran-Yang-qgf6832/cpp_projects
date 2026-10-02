// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务路由的实现——分派、统一校验、以及动力学收尾。

#include "router/kdl_task_router.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <urdf/model.h>

#include "tasks/kdl_cartesian_space_task.hpp"
#include "kdl_idynamics.hpp"
#include "tasks/kdl_joint_space_task.hpp"
#include "kdl_tools.hpp"

namespace kdl_control
{
namespace
{

/// 力矩前馈的采样点数上限：sample_dt 被误设得很小时不至于把内存撑爆。
constexpr unsigned int kMaxFeedforwardSamples = 5000;

/// 采样步长的兜底值 [s]（与 ros2_control 默认 100 Hz 控制周期一致）。
constexpr double kDefaultSampleDt = 0.01;

/// 把数值格式化成可读字符串（只用于拼错误信息，故各 .cpp 各自持有这一份小工具）。
std::string num(double value, int precision = 6)
{
  std::ostringstream os;
  os << std::setprecision(precision) << value;
  return os.str();
}

}  // namespace

TaskRouter::TaskRouter()
{
  // 内置任务在这里注册；后续新增任务只需三步：TaskType 加枚举、写实现类、
  // 在这里 registerTask（或者由调用者自己注入）。
  registerTask(std::make_shared<JointSpaceTask>());
  registerTask(std::make_shared<CartesianSpaceTask>());
}

TaskRouter::~TaskRouter() = default;

void TaskRouter::setContext(const RobotContext & ctx) { ctx_ = ctx; }

void TaskRouter::registerTask(std::shared_ptr<TaskBase> task)
{
  if (!task) {
    return;
  }
  tasks_[task->type()] = std::move(task);
}

bool TaskRouter::hasTask(TaskType type) const { return tasks_.find(type) != tasks_.end(); }

const char * TaskRouter::taskName(TaskType type) const
{
  const auto it = tasks_.find(type);
  return it == tasks_.end() ? "unimplemented" : it->second->name();
}

bool TaskRouter::loadContextFromUrdf(
  const std::string & urdf_file, RobotContext & ctx, std::string & message,
  const std::string & base_link, const std::string & tip_link)
{
  // 失败时把上下文清空：调用者拿到"半个上下文"只会带来更难查的问题。
  ctx = RobotContext{};
  message.clear();

  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain, base_link, tip_link)) {
    message = "从 URDF 建链失败：" + urdf_file + "（base = " + base_link + ", tip = " + tip_link +
              "）；请确认文件存在、两个 link 名字正确且互相连通";
    return false;
  }

  urdf::Model model;
  if (!kdl_tools::loadUrdfModel(urdf_file, model)) {
    message = "解析 URDF 失败（读不到质量/惯量/限位信息）：" + urdf_file;
    return false;
  }

  const unsigned int n = chain.getNrOfJoints();
  ctx.chain = chain;
  ctx.q_min.resize(n);
  ctx.q_max.resize(n);
  ctx.max_torque.resize(n);

  // 与 KDL 的编号规则保持一致：固定段不占自由度，所以按"段的关节不是 None"来计数。
  unsigned int index = 0;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // 固定段
    }

    const auto it = model.joints_.find(joint.getName());
    if (it == model.joints_.end() || !it->second->limits) {
      message = "关节 '" + joint.getName() +
                "' 在 URDF 中缺少 <limit>（lower/upper/effort）：行程与力矩上限都读不到";
      ctx = RobotContext{};
      return false;
    }

    ctx.q_min(index) = it->second->limits->lower;
    ctx.q_max(index) = it->second->limits->upper;
    ctx.max_torque(index) = it->second->limits->effort;  // 0 表示不校验力矩
    ++index;
  }

  if (index != n) {
    message = "关节限位个数（" + std::to_string(index) + "）与链的可动关节数（" +
              std::to_string(n) + "）不一致：URDF 里可能有 KDL 无法表达的关节类型";
    ctx = RobotContext{};
    return false;
  }

  // 速度/加速度/jerk 与笛卡尔限位留给调用者显式设置（见头文件注释）。
  return ctx.valid();
}

bool TaskRouter::checkDynamicsAvailable(const RobotContext & ctx, std::string & message)
{
  const unsigned int n = ctx.chain.getNrOfJoints();
  if (n == 0) {
    message = "链上没有可动关节，动力学无意义";
    return false;
  }

  // 探针位形取行程中点（行程未知时退化为零位），保证不会跑到离谱的构型上。
  KDL::JntArray q(n);
  KDL::JntArray qdot(n);
  KDL::JntArray qddot(n);
  const bool has_limits = (ctx.q_min.rows() == n && ctx.q_max.rows() == n);
  for (unsigned int i = 0; i < n; ++i) {
    q(i) = has_limits ? 0.5 * (ctx.q_min(i) + ctx.q_max(i)) : 0.0;
    qdot(i) = 0.0;
    // 给一个 1 rad/s² 的假想加速度：τ = M(q)·q̈，只要链上有质量，τ 就不该是 0。
    // 刻意不用重力项做判据：全是竖直轴的机器人（如 SCARA）重力项本来恒为 0，
    // 那会把"有惯量"误判成"没惯量"。
    qddot(i) = 1.0;
  }

  const kdl_dynamics::IdResult id = kdl_dynamics::inverseDynamics(ctx.chain, q, qdot, qddot);
  if (!id.success()) {
    message = "逆动力学求解失败：" + id.message;
    return false;
  }

  double max_torque = 0.0;
  for (unsigned int i = 0; i < n; ++i) {
    max_torque = std::max(max_torque, std::abs(id.torque(i)));
  }
  if (max_torque < 1e-9) {
    message = "动力学结果恒为 0：链上各段很可能没有惯量参数（URDF 的 <link> 缺少 "
              "<inertial>），此时力矩前馈没有意义，请先补全惯量参数";
    return false;
  }

  return true;
}

ControlResult TaskRouter::dispatch(
  const TaskRequest & req, const KDL::JntArray & q_now, const KDL::JntArray & qdot_now) const
{
  ControlResult result;
  result.type = req.type;

  // ---- 1. 上下文 ----
  if (!ctx_.valid()) {
    result.message =
      "机器人上下文未装配或非法：请先调用 setContext() 或 loadContextFromUrdf()";
    return result;
  }

  // ---- 2. 分派 ----
  const auto it = tasks_.find(req.type);
  if (it == tasks_.end() || !it->second) {
    result.message = std::string("任务类型未实现：") + taskTypeName(req.type) +
                     "（新增任务需要：TaskType 加枚举 + 写 TaskBase 派生类 + registerTask）";
    return result;
  }
  const TaskBase & task = *it->second;

  // ---- 3. 统一校验：状态向量长度 ----
  const unsigned int n = ctx_.chain.getNrOfJoints();
  if (q_now.rows() != n) {
    result.message = "当前关节角长度 " + std::to_string(q_now.rows()) + " 与链的关节数 " +
                     std::to_string(n) + " 不匹配";
    return result;
  }
  if (qdot_now.rows() != 0 && qdot_now.rows() != n) {
    result.message = "当前关节速度长度 " + std::to_string(qdot_now.rows()) +
                     " 既不等于关节数 " + std::to_string(n) + "，也不为空";
    return result;
  }

  // ---- 4. 任务自己的前置校验 ----
  std::string message;
  if (!task.validate(ctx_, req, message)) {
    result.message = message;
    return result;
  }

  // ---- 5. 解算 ----
  result = task.solve(ctx_, req, q_now, qdot_now);
  result.type = req.type;
  if (!result.success) {
    return result;
  }

  // ---- 6. 横切收尾：动力学前馈 / 力矩上限 ----
  if (req.compute_torque_feedforward || req.check_torque_limit) {
    if (!computeTorqueFeedforward(req, result, message)) {
      // 轨迹本身是好的，失败只针对"力矩可行性"；轨迹仍保留在结果里（见
      // ControlResult::joint_trajectory 的注释），方便调用者降速重试或直接分析。
      result.success = false;
      result.message = message;
    }
  }

  return result;
}

bool TaskRouter::computeTorqueFeedforward(
  const TaskRequest & req, ControlResult & result, std::string & message) const
{
  const unsigned int n = ctx_.chain.getNrOfJoints();

  // 先确认链上真的有惯量参数，否则后面会安静地得到一堆 0（比报错更难查）。
  if (!checkDynamicsAvailable(ctx_, message)) {
    return false;
  }

  const double total = result.joint_trajectory.duration();
  if (total <= 0.0) {
    message = "轨迹总时长为 0，无法计算力矩前馈";
    return false;
  }

  const double dt_requested = req.sample_dt > 0.0 ? req.sample_dt : kDefaultSampleDt;
  unsigned int intervals = static_cast<unsigned int>(std::ceil(total / dt_requested));
  if (intervals < 1) {
    intervals = 1;
  }
  if (intervals + 1 > kMaxFeedforwardSamples) {
    intervals = kMaxFeedforwardSamples - 1;
  }
  const double dt = total / static_cast<double>(intervals);

  result.torque_feedforward.clear();
  result.torque_times.clear();
  result.torque_feedforward.reserve(intervals + 1);
  result.torque_times.reserve(intervals + 1);

  // 只有当上限数组齐全时才做判定：长度 0 是"不校验"，不是"上限为 0"。
  const bool check_limit = req.check_torque_limit && (ctx_.max_torque.rows() == n);

  for (unsigned int k = 0; k <= intervals; ++k) {
    const double t = (k == intervals) ? total : dt * static_cast<double>(k);

    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qddot;
    if (!result.joint_trajectory.sample(t, q, qdot, qddot)) {
      message = "轨迹采样失败（t = " + num(t) + " s），无法计算力矩前馈";
      return false;
    }

    const kdl_dynamics::IdResult id = kdl_dynamics::inverseDynamics(ctx_.chain, q, qdot, qddot);
    if (!id.success()) {
      message = "逆动力学求解失败（t = " + num(t) + " s）：" + id.message;
      return false;
    }

    if (check_limit) {
      for (unsigned int i = 0; i < n; ++i) {
        if (std::abs(id.torque(i)) > ctx_.max_torque(i)) {
          message = "力矩超限：t = " + num(t) + " s，第 " + std::to_string(i + 1) +
                    " 个关节需要 " + num(id.torque(i)) + " N·m，上限 " +
                    num(ctx_.max_torque(i)) + " N·m；请放慢轨迹或减轻负载";
          return false;
        }
      }
    }

    result.torque_feedforward.push_back(id.torque);
    result.torque_times.push_back(t);
  }

  return true;
}

}  // namespace kdl_control
