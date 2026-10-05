// Copyright (c) 2026, kdl_tools authors.
// 教学用途：kdl_state_machine.hpp 的全部实现（本模块只有一个头文件 + 本实现 + 一个 main）。
//
// 读这个文件的顺序建议：
//   1) 第二行的"任务发送"一节 —— taskKindName / buildTarget / buildRequest / sendTask /
//      pollTask：自由函数 + 调用者持有的 TaskHandle，不依赖状态机；
//   2) KdlStateMachine 的 startTask() / startJointTask() / startCartesianTask()：
//      "数值数组 → 任务目标 → 装成只发一条的来源"；
//   3) stepSendTask() —— 状态机里唯一涉及任务的函数：向 TaskSource 要目标 →
//      sendTask() / pollTask() → 把结果翻译成下一个状态；
//   4) tick() / setState() —— 迁移的驱动与唯一出口。
//
// 本文件里**没有**任何"任务链条"：链条由 TaskSource 决定
// （startTask() 装的是单条；多步链条靠 setTaskSource() 注入）。

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "kdl_state_machine.hpp"

namespace
{

using ControlTask = kdl_tools::srv::ControlTask;

/// 打印用：把一串数写成 [a, b, c]。
std::string formatVector(const std::vector<double> & values)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    os << (i == 0 ? "" : ", ") << values[i];
  }
  os << "]";
  return os.str();
}

/// 打印用：把一条任务目标写成一行（日志里要能一眼看出"这条任务要干什么"）。
std::string describeTarget(const kdl_state_machine::TaskTarget & target)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << kdl_state_machine::taskKindName(target.kind);
  if (target.kind == kdl_state_machine::TaskKind::kJointSpace) {
    os << " goal=" << formatVector(target.joint_goal);
  } else if (target.kind == kdl_state_machine::TaskKind::kCartesianSpace) {
    os << " p=[" << target.pose.position.x << ", " << target.pose.position.y << ", "
       << target.pose.position.z << "] q=[" << target.pose.orientation.x << ", "
       << target.pose.orientation.y << ", " << target.pose.orientation.z << ", "
       << target.pose.orientation.w << "]";
    // 预留：新增任务类型时在这里加一段打印
  }
  os << " duration=" << target.duration << " s";
  return os.str();
}

/// 统一的 Trigger 响应填充。
void fillTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Response> & response, bool success,
  const std::string & message)
{
  response->success = success;
  response->message = message;
}

}  // namespace

namespace kdl_state_machine
{

// ===========================================================================
// 二、任务发送：自由函数 + 调用者持有的句柄
// ===========================================================================

const char * taskKindName(TaskKind kind)
{
  switch (kind) {
    case TaskKind::kJointSpace: return "joint_space";
    case TaskKind::kCartesianSpace: return "cartesian_space";
    // ---- 预留：新增任务类型时在这里加名字 ----
  }
  return "unknown";
}

bool buildTarget(
  TaskKind kind, const std::vector<double> & values, TaskTarget & target, std::string & message)
{
  target = TaskTarget{};  // 从干净状态开始，避免残留上次的字段

  // 数组怎么解释，全由 kind 决定。新增任务类型时在这里加 case。
  switch (kind) {
    case TaskKind::kJointSpace: {
      // [q1, q2, ..., qn, duration]：至少 2 个数（1 个关节 + 时长）。
      if (values.size() < 2) {
        message = "关节空间任务需要 [q1, ..., qn, duration]，至少 2 个数（当前 " +
                  std::to_string(values.size()) + " 个）";
        return false;
      }
      const double duration = values.back();
      if (!std::isfinite(duration)) {
        message = "任务时长必须是有限值（当前 " + std::to_string(duration) + "）";
        return false;
      }
      target.kind = TaskKind::kJointSpace;
      target.joint_goal.assign(values.begin(), values.end() - 1);  // 去掉末位的时长
      target.duration = duration;
      return true;
    }
    case TaskKind::kCartesianSpace: {
      // [x, y, z, qx, qy, qz, qw, duration]：固定 8 个数。
      if (values.size() != 8) {
        message = "笛卡尔空间任务需要 [x, y, z, qx, qy, qz, qw, duration] 共 8 个数（当前 " +
                  std::to_string(values.size()) + " 个）";
        return false;
      }
      const double duration = values[7];
      if (!std::isfinite(duration)) {
        message = "任务时长必须是有限值（当前 " + std::to_string(duration) + "）";
        return false;
      }
      target.kind = TaskKind::kCartesianSpace;
      // 四元数不必归一化：control 层会自己归一（范数为 0 时退化成单位旋转）。
      target.pose.position.x = values[0];
      target.pose.position.y = values[1];
      target.pose.position.z = values[2];
      target.pose.orientation.x = values[3];
      target.pose.orientation.y = values[4];
      target.pose.orientation.z = values[5];
      target.pose.orientation.w = values[6];
      target.duration = duration;
      return true;
    }
      // ---- 预留：新增任务类型时在这里加 case ----
  }

  // 走到这里说明 kind 是"没实现/越界"的值：如实报错，不猜。
  message = "未知任务类型（TaskKind = " + std::to_string(static_cast<int>(kind)) + "）";
  return false;
}

bool buildRequest(const TaskTarget & target, ControlTask::Request & request, std::string & message)
{
  request = ControlTask::Request{};  // 从干净状态开始，避免残留上次的字段
  request.duration = target.duration;

  // 任务类型的分派就在这个 switch 里（新增任务类型的唯一落点）。
  switch (target.kind) {
    case TaskKind::kJointSpace: {
      if (target.joint_goal.empty()) {
        message = "关节空间任务缺少 joint_goal（长度应为机器人的关节数）";
        return false;
      }
      request.task_type = ControlTask::Request::JOINT_SPACE;
      request.goal_joint = target.joint_goal;
      return true;
    }
    case TaskKind::kCartesianSpace: {
      request.task_type = ControlTask::Request::CARTESIAN_SPACE;
      // 四元数不必归一化：control 层会自己归一（范数为 0 时退化成单位旋转）。
      request.goal_pose = target.pose;
      return true;
    }
      // ---- 预留：新增任务类型时在这里加 case ----
      // case TaskKind::kCircular: {
      //   request.task_type = ...;   // 需要给 ControlTask.srv 补字段
      //   return true;
      // }
  }

  // 走到这里说明 kind 是"没实现/越界"的值：如实报错，不猜。
  message = "未知任务类型（TaskKind = " + std::to_string(static_cast<int>(target.kind)) + "）";
  return false;
}

SendStatus sendTask(TaskHandle & handle, const TaskTarget & target, std::string & message)
{
  if (taskInFlight(handle)) {
    message = "已有任务在飞：一个 TaskHandle 一次只承载一条（control 层也不支持并发）";
    return SendStatus::kBusy;
  }

  // 先翻译目标，再查服务：这样"目标非法"不会因为服务没起来而被报成 kNoService。
  ControlTask::Request request;
  if (!buildRequest(target, request, message)) {
    return SendStatus::kBadTarget;
  }
  if (!serviceReady(handle)) {
    message = "service /control_task 不可用（client 为空，或 control 节点没起来）";
    return SendStatus::kNoService;
  }

  // 注意 .future.share()：async_send_request 返回的是 FutureAndRequestId，
  // 它到 SharedFuture 的**隐式**转换在 Jazzy 已标记 deprecated，显式取 future 再 share。
  handle.pending =
    handle.client->async_send_request(std::make_shared<ControlTask::Request>(request))
      .future.share();
  return SendStatus::kOk;
}

PollStatus pollTask(TaskHandle & handle, TaskResult & result)
{
  if (!handle.pending.valid()) {
    return PollStatus::kIdle;  // 没有在飞的任务
  }
  if (handle.pending.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return PollStatus::kRunning;  // 服务端还在执行轨迹（它要等执行完才返回）
  }

  const ControlTask::Response::SharedPtr response = handle.pending.get();
  handle.pending = {};  // 复位：允许下一次 sendTask()

  result = TaskResult{};
  if (!response) {
    result.message = "service 响应为空";
    return PollStatus::kReady;
  }
  result.error_code = response->error_code;
  result.success = (response->error_code == 0) && response->success;
  result.message = response->message;
  result.trajectory_duration = response->trajectory_duration;
  return PollStatus::kReady;
}

void discardTask(TaskHandle & handle)
{
  // 只丢掉等待：请求已经发出（服务端会把它执行完），这里不再关心结果。
  handle.pending = {};
}

bool taskInFlight(const TaskHandle & handle)
{
  return handle.pending.valid();
}

bool serviceReady(const TaskHandle & handle)
{
  return handle.client && handle.client->service_is_ready();
}

// ===========================================================================
// 三、任务状态机
// ===========================================================================

const char * stateName(State state)
{
  switch (state) {
    case State::kIdle: return "idle";
    case State::kWaitForSystem: return "wait_for_system";
    case State::kSendTask: return "send_task";
    case State::kFinished: return "finished";
    case State::kFailed: return "failed";
  }
  return "unknown";
}

// ---- 装配 ----

KdlStateMachine::KdlStateMachine(const rclcpp::NodeOptions & options)
: rclcpp::Node("state_machine", options)
{
  std::string message;
  if (!readParameters(message)) {
    throw std::runtime_error("参数错误：" + message);
  }
  if (!setupInterfaces(message)) {
    throw std::runtime_error("建立接口失败：" + message);
  }

  // 构造函数不装任务来源：由 startTask() / startJointTask() / startCartesianTask()
  // 或 setTaskSource() 在启动前指定（状态机自己**不定义链条**）。
  printPlan();
  publishState();  // 话题是 transient_local：晚启动的订阅者也能看到初始状态
}

void KdlStateMachine::setTaskSource(TaskSource source)
{
  task_source_ = std::move(source);
  RCLCPP_INFO(
    get_logger(), "已设置自定义任务来源（会覆盖 startTask() 装的那一条）%s",
    task_source_ ? "" : "；注意：来源为空，状态机会直接进 finished");
}

bool KdlStateMachine::startJointTask(const std::vector<double> & values, std::string & message)
{
  return startTask(TaskKind::kJointSpace, values, message);
}

bool KdlStateMachine::startCartesianTask(const std::vector<double> & values, std::string & message)
{
  return startTask(TaskKind::kCartesianSpace, values, message);
}

bool KdlStateMachine::startTask(
  TaskKind kind, const std::vector<double> & values, std::string & message)
{
  // 接受条件与 ~/start 一致：运行中不接受，避免中途替换任务导致参考跳变。
  if (!canStart(message)) {
    return false;
  }

  // 先把数值数组翻译成目标：非法就在这里挡掉，不去动状态机。
  TaskTarget target;
  if (!buildTarget(kind, values, target, message)) {
    return false;  // message 已由 buildTarget 填好
  }

  // 装一条"只给一次"的来源。本轮跑完（kFinished）后要再跑，调 ~/start 即可：
  // resetRun() 会清 one_shot_used_，同一条任务会再发一次。
  task_source_ = [this, target](TaskTarget & out) -> bool {
      if (one_shot_used_) {
        return false;  // 已经给过 → 状态机进 kFinished
      }
      one_shot_used_ = true;
      out = target;
      return true;
    };

  beginRun(message);  // 一定会成功（上面刚用 canStart 判过）
  RCLCPP_INFO(get_logger(), "启动单条任务：%s", describeTarget(target).c_str());
  return true;
}

bool KdlStateMachine::readParameters(std::string & message)
{
  service_name_ = declare_parameter<std::string>("service_name", "/control_task");
  status_topic_ = declare_parameter<std::string>(
    "status_topic", "/kdl_effort_controller/status");
  joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");

  auto_start_ = declare_parameter<bool>("auto_start", true);
  tick_period_ = declare_parameter<double>("tick_period", 0.05);
  system_timeout_ = declare_parameter<double>("system_timeout", 60.0);
  // 依赖凑齐后再等这么久才发第一条任务：给 control 节点一点时间处理完
  // /joint_states 的第一帧（否则可能被它回 error_code = 1）。
  start_delay_ = declare_parameter<double>("start_delay", 0.5);

  if (!(tick_period_ > 0.0) || !std::isfinite(tick_period_)) {
    message = "tick_period 必须是正数（当前 " + std::to_string(tick_period_) + "）";
    return false;
  }
  if (!(system_timeout_ > 0.0) || !std::isfinite(system_timeout_)) {
    message = "system_timeout 必须是正数（当前 " + std::to_string(system_timeout_) + "）";
    return false;
  }
  if (!(start_delay_ >= 0.0) || !std::isfinite(start_delay_)) {
    message = "start_delay 不能是负数（当前 " + std::to_string(start_delay_) + "）";
    return false;
  }
  return true;
}

bool KdlStateMachine::setupInterfaces(std::string & message)
{
  // 句柄的 client 由本节点创建；之后发送/收结果都用头文件里那几个自由函数。
  handle_.client = create_client<ControlTask>(service_name_);
  if (!handle_.client) {
    message = "创建 service client 失败：" + service_name_;
    return false;
  }

  // ~/state：keep_last(1) + transient_local，晚订阅的人也能立刻看到当前状态。
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "~/state", rclcpp::QoS(1).transient_local().reliable());

  // 控制器状态只用来判断"控制器是否已经激活"这一件事：收到过一帧就够。
  // 用 best_effort 订阅 reliable 发布者是兼容的（反过来不保证），
  // 与 kdl_control_node 订阅 /joint_states 的取舍一致。
  status_sub_ = create_subscription<ControlStatus>(
    status_topic_, rclcpp::QoS(10).best_effort(),
    std::bind(&KdlStateMachine::statusCallback, this, std::placeholders::_1));

  // /joint_states 只用来判断"状态广播器是否已经 active"。必须查它：
  // joint_state_broadcaster 与 kdl_effort_controller 是两个 spawner 并行加载的，
  // 控制器可能先 active，而 control 节点此时还拿不到关节状态、会回 error_code = 1。
  // QoS 与 kdl_control_node 保持一致（SensorDataQoS 语义：best_effort）。
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&KdlStateMachine::jointStatesCallback, this, std::placeholders::_1));

  start_srv_ = create_service<Trigger>(
    "~/start",
    std::bind(
      &KdlStateMachine::startCallback, this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = create_service<Trigger>(
    "~/stop",
    std::bind(
      &KdlStateMachine::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

  // **墙钟**定时器，不是仿真时钟：状态机是上层调度，仿真被 ~/set_pause 暂停时
  // 它仍要能观察到"等依赖超时"、仍要能响应 ~/stop，不能跟着仿真时钟停摆。
  // （与 kdl_control_node 的"等执行结果用墙钟"同一口径。）
  tick_timer_ = create_wall_timer(
    std::chrono::duration<double>(tick_period_),
    std::bind(&KdlStateMachine::tick, this));
  return true;
}

void KdlStateMachine::printPlan() const
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(3);
  os << "  自动开始   : " << (auto_start_ ? "true（就绪后自动跑，可用 ~/stop 打断）" : "false（等 ros2 service call ~/start）") << "\n";
  os << "  等依赖上限 : " << system_timeout_ << " s，就绪后再等 " << start_delay_
     << " s（墙钟）\n";
  os << "  依赖       : service=" << service_name_ << "，status=" << status_topic_
     << "，state=" << joint_states_topic_ << "\n";
  os << "  任务来源   : 未设置 —— 用 startJointTask() / startCartesianTask() / startTask()\n";
  os << "               指定一条，或用 setTaskSource() 注入链条；都没设会直接进 finished\n";

  RCLCPP_INFO(
    get_logger(),
    "任务状态机就绪：每 %.0f ms 一拍，service=%s，状态话题=~/state\n%s\n"
    "  状态机不定义任务链条：wait_for_system → send_task（向来源要任务）→ finished/failed",
    tick_period_ * 1000.0, service_name_.c_str(), os.str().c_str());
}

// ---- 状态处理函数 ----

State KdlStateMachine::stepIdle()
{
  // auto_start 只生效一次：手工 ~/stop 回到 idle 后不会被它立刻又拉起来。
  if (!auto_start_ || auto_started_) {
    return State::kIdle;
  }
  auto_started_ = true;
  return State::kWaitForSystem;
}

State KdlStateMachine::stepWaitForSystem()
{
  if (!wait_started_) {
    wait_started_ = true;
    wait_start_ = std::chrono::steady_clock::now();
  }

  const auto now = std::chrono::steady_clock::now();
  const bool service_ok = serviceReady(handle_);
  const bool deps_ok = service_ok && status_seen_ && joint_states_seen_;

  if (deps_ok) {
    if (!all_ready_) {
      all_ready_ = true;
      all_ready_at_ = now;
      RCLCPP_INFO(
        get_logger(),
        "依赖就绪：service %s 可见、控制器状态已收到、%s 已收到；再等 %.2f s 开始发任务",
        service_name_.c_str(), joint_states_topic_.c_str(), start_delay_);
    }
    if (std::chrono::duration<double>(now - all_ready_at_).count() >= start_delay_) {
      return State::kSendTask;
    }
    return State::kWaitForSystem;
  }

  // 依赖中途掉了（例如广播器重启）→ 重新计时，不能让"曾经凑齐过"一直成立。
  all_ready_ = false;

  const double waited = std::chrono::duration<double>(now - wait_start_).count();
  if (waited > system_timeout_) {
    return fail(
      "等待依赖超时（" + std::to_string(waited) + " s）：service " + service_name_ +
      (service_ok ? " 已就绪" : " 不可见") + "；控制器状态话题 " + status_topic_ +
      (status_seen_ ? " 已收到" : " 未收到") + "；" + joint_states_topic_ +
      (joint_states_seen_ ? " 已收到" : " 未收到") +
      "。请确认 kdl_control_node 起了、joint_state_broadcaster 与 kdl_effort_controller 都已 active");
  }
  return State::kWaitForSystem;
}

State KdlStateMachine::stepSendTask()
{
  // ---- (1) 有任务在飞：取一次结果 ----
  if (taskInFlight(handle_)) {
    TaskResult result;
    switch (pollTask(handle_, result)) {
      case PollStatus::kRunning:
        return State::kSendTask;  // 服务端还在跑轨迹，下一拍再查
      case PollStatus::kIdle:
        return State::kSendTask;  // 理论上到不了（taskInFlight 为真时一定有在飞的任务）
      case PollStatus::kReady:
        break;                    // 结果到手，下面按成功/失败分流
    }

    if (!result.success) {
      return fail(
        "任务失败：error_code = " + std::to_string(result.error_code) + "（" +
        result.message + "）");
    }
    ++completed_steps_;
    RCLCPP_INFO(
      get_logger(), "任务完成（累计 %u 条）：%s", completed_steps_, result.message.c_str());
    return State::kSendTask;  // 继续向任务来源要下一条
  }

  // ---- (2) 没有在飞的任务：向"任务来源"要一条 ----
  // 目标全部来自 task_source_（由 startTask() / setTaskSource() 设置）；
  // 状态机自己**不产生**任何目标，也不判断任务类型。
  TaskTarget target;
  if (!task_source_) {
    RCLCPP_WARN(
      get_logger(),
      "没有任务来源（未调 startTask() / setTaskSource()），直接进 finished；共完成 %u 条",
      completed_steps_);
    return State::kFinished;
  }
  if (!task_source_(target)) {
    RCLCPP_INFO(get_logger(), "任务来源说没有更多任务，共完成 %u 条", completed_steps_);
    return State::kFinished;
  }

  std::string message;
  const SendStatus status = sendTask(handle_, target, message);
  if (status != SendStatus::kOk) {
    return fail("下发任务失败：" + message);
  }
  RCLCPP_INFO(
    get_logger(), "已下发任务 #%u：%s", completed_steps_ + 1, describeTarget(target).c_str());
  return State::kSendTask;  // 下一拍开始查结果
}

State KdlStateMachine::fail(const std::string & message)
{
  fail_message_ = message;
  RCLCPP_ERROR(get_logger(), "任务序列失败：%s", message.c_str());
  return State::kFailed;
}

// ---- 驱动与迁移 ----

void KdlStateMachine::tick()
{
  switch (state_) {
    case State::kIdle:
      setState(stepIdle());
      return;
    case State::kWaitForSystem:
      setState(stepWaitForSystem());
      return;
    case State::kSendTask:
      setState(stepSendTask());
      return;
    case State::kFinished:
    case State::kFailed:
      return;  // 终态：等 ~/start（或 ~/stop 明确回到 idle）
  }
}

void KdlStateMachine::setState(State next)
{
  if (state_ == next) {
    return;
  }
  RCLCPP_INFO(get_logger(), "状态迁移：%s → %s", stateName(state_), stateName(next));
  state_ = next;
  publishState();

  if (state_ == State::kFinished) {
    RCLCPP_INFO(
      get_logger(), "任务序列全部完成（共 %u 条）。要再跑一遍就调 ~/start。", completed_steps_);
  }
}

void KdlStateMachine::publishState()
{
  if (!state_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = stateName(state_);
  if (state_ == State::kFailed && !fail_message_.empty()) {
    msg.data += "：" + fail_message_;  // 失败时把原因一起发出去，订阅者不用翻日志
  }
  state_pub_->publish(msg);
}

void KdlStateMachine::resetRun()
{
  fail_message_.clear();
  completed_steps_ = 0;
  wait_started_ = false;
  all_ready_ = false;      // 重新走一遍"等依赖 + start_delay"的流程
  one_shot_used_ = false;  // startTask() 装的那条可以再给一次（自定义来源自己负责重跑语义）
}

bool KdlStateMachine::canStart(std::string & message) const
{
  if (state_ == State::kIdle || state_ == State::kFinished || state_ == State::kFailed) {
    return true;  // 空闲或终态：可以开始新的一轮
  }
  message = std::string("状态机正在运行（当前 ") + stateName(state_) + "），先调 ~/stop 或等它结束";
  return false;
}

bool KdlStateMachine::beginRun(std::string & message)
{
  if (!canStart(message)) {
    return false;
  }
  resetRun();
  auto_started_ = true;  // 免得 auto_start 再来插一脚（与 ~/start 的处理一致）
  setState(State::kWaitForSystem);
  return true;
}

// ---- 回调 ----

void KdlStateMachine::statusCallback(const ControlStatus::SharedPtr msg)
{
  if (status_seen_) {
    return;
  }
  // 控制器只在 update() 被调用（= 已激活）之后才会发这个话题，
  // 所以"收到过一帧"就是"控制器已经 active"的充分证据。
  status_seen_ = true;
  RCLCPP_INFO(
    get_logger(), "收到控制器状态（控制器已激活）：active=%s，message=%s",
    msg->active ? "true" : "false", msg->message.c_str());
}

void KdlStateMachine::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (joint_states_seen_) {
    return;
  }
  joint_states_seen_ = true;
  RCLCPP_INFO(
    get_logger(), "收到 %s（%zu 个关节，状态广播器已 active）", joint_states_topic_.c_str(),
    msg->name.size());
}

void KdlStateMachine::startCallback(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  std::string message;
  if (!beginRun(message)) {
    fillTrigger(response, false, message);
    return;
  }
  fillTrigger(response, true, std::string("已启动，当前状态 ") + stateName(state_));
}

void KdlStateMachine::stopCallback(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (state_ == State::kIdle || state_ == State::kFinished || state_ == State::kFailed) {
    fillTrigger(
      response, false, std::string("状态机不在运行（当前 ") + stateName(state_) + "）");
    return;
  }

  std::string note = "已停止推进后续步骤";
  if (taskInFlight(handle_)) {
    // control 层的 service 语义是"把这条轨迹执行完再返回"，没有取消接口，
    // 所以只能丢掉等待：那条轨迹仍会被控制器跑完。
    note += "；注意：已下发的那条任务仍会被执行完（control 层不支持取消），"
            "要重新启动请等它跑完，否则新任务会被回 error_code = 7";
  }
  discardTask(handle_);
  setState(State::kIdle);
  fillTrigger(response, true, note);
}

}  // namespace kdl_state_machine
