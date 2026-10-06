// Copyright (c) 2026, kdl_tools authors.
// 教学用途：任务解算节点（control 层）——service → 解算 → 离散 → /control_reference。
//
// ===========================================================================
// 这一层在整条链路里的位置
// ===========================================================================
//   [调用者] ──srv /control_task──▶ 本节点
//                                   │ ① /joint_states 取起点 q_now
//                                   │ ② TaskRouter::dispatch()  → 关节五次轨迹 (C⁴)
//                                   │ ③ 按 point_dt 采样 (q,q̇,q̈) → inverseDynamics → τ_ff
//                                   │ ④ 离散成 JointTrajectory(position/velocity/effort)
//                                   │ ⑤ publish /control_reference
//                                   │ ⑥ 等控制器 ~/status 的 active:false → 回 response
//                                   ▼
//                     topic /control_reference
//                                   ▼
//              kdl_effort_controller（CM 500 Hz update 内算力矩）
//
// 与旧实现（已退休的 control_bridge_node）的区别：
//   旧：解算后采样给 JTC 发 action，用 position 命令接口；
//   新：解算后按控制周期离散成"控制点"（position/velocity/effort）发话题，
//       由力矩控制器用来做"前馈 + 反馈"。前馈 τ_ff 在这里逐采样点算好。
//
// 为什么前馈在这里逐点算、而不用 TaskRouter::compute_torque_feedforward：
//   后者按 sample_dt 采样且有点数上限，与本节点的 point_dt 网格不保证对齐；
//   逐点直接调用 kdl_dynamics::inverseDynamics 简单、精确、无耦合。
// ===========================================================================
//
// 三个"不写下来就会被坑"的实现细节：
//   1) **必须多线程执行器**：service 回调会阻塞着等轨迹执行完（几秒），单线程
//      执行器会让 /joint_states 订阅与 status 回调一起停摆，表现为"service 永不返回"。
//   2) **关节顺序按名字对齐**：TaskRouter 输出是 KDL 链的顺序，JointTrajectory 要求
//      的顺序是控制器（controllers.yaml）的顺序，两者不保证相同。
//   3) **URDF 限位里没有速度/加速度**：自动定时（duration <= 0）与轨迹可行性校验都
//      依赖 ctx.joint_limits，而 loadContextFromUrdf 不填它，必须由本节点按参数显式给出。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_idynamics.hpp"
#include "kdl_tools/msg/control_status.hpp"
#include "kdl_tools/srv/control_task.hpp"
#include "router/kdl_task_router.hpp"

namespace
{

/// 本节点自己的错误码：正数留给本节点，负数留给控制器/硬件（见 ControlTask.srv 注释）。
enum NodeError : int32_t
{
  kOk = 0,
  kNoJointState = 1,
  kBadRequest = 2,
  kSolveFailed = 3,
  kBadConfig = 4,
  kControllerNotActive = 5,
  kTimeout = 6,
  kBusy = 7,
  kNotReached = 8
};

/// 保护上限：防止"duration 给了个巨大的值"把消息撑爆（DDS 单条消息有限制）。
constexpr std::size_t kDefaultMaxTrajectoryPoints = 50000;

/// geometry_msgs/Pose → KDL::Frame。
/// 先归一化四元数：KDL::Rotation::Quaternion 在部分版本里不检查范数，
/// 传进来的四元数没归一化会构造出带缩放的"旋转矩阵"（行列式 != 1）。
KDL::Frame frameFromPose(const geometry_msgs::msg::Pose & pose)
{
  const double x = pose.orientation.x;
  const double y = pose.orientation.y;
  const double z = pose.orientation.z;
  const double w = pose.orientation.w;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    // 范数为 0 / 非有限 → 单位旋转，避免产生 NaN 位姿后一路传到 IK 里
    return KDL::Frame(
      KDL::Rotation::Identity(),
      KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
  }
  return KDL::Frame(
    KDL::Rotation::Quaternion(x / norm, y / norm, z / norm, w / norm),
    KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

/// 把参数里的限位数组填进 KDL::JntArray。
/// 长度 0 = "调用者不要求校验这一项"（与 kdl_interpolation 的约定一致）；
/// 非空但长度不等于关节数 = 配置错误，直接报出来，不要静默忽略。
bool fillLimits(
  const std::vector<double> & from_param, unsigned int n_joints, KDL::JntArray & out,
  const char * param_name, std::string & message)
{
  if (from_param.empty()) {
    out = KDL::JntArray(0);
    return true;
  }
  if (from_param.size() != n_joints) {
    message = std::string(param_name) + " 的长度 " + std::to_string(from_param.size()) +
              " 与关节数 " + std::to_string(n_joints) + " 不匹配";
    return false;
  }
  out.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i) {
    out(i) = from_param[i];
  }
  return true;
}

}  // namespace

/**
 * @brief 任务解算节点：把 service 请求解算成一条带前馈力矩的关节轨迹并发布。
 */
class ControlNode : public rclcpp::Node
{
public:
  using ControlTask = kdl_tools::srv::ControlTask;
  using ControlStatus = kdl_tools::msg::ControlStatus;

  ControlNode();

private:
  // -------------------------------------------------------------------------
  // 装配
  // -------------------------------------------------------------------------

  /// 读参数（含默认值），返回是否全部合法。
  bool declareAndReadParameters(std::string & message);

  /// 用 urdf_file 装配 RobotContext，并把速度/加速度/jerk 上限填进去。
  bool setupContext(std::string & message);

  /// 建立"控制器关节名 ↔ KDL 链关节下标"的映射（顺序无关）。
  bool setupJointMapping(std::string & message);

  // -------------------------------------------------------------------------
  // 运行期
  // -------------------------------------------------------------------------

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void statusCallback(const ControlStatus::SharedPtr msg);

  void controlTaskCallback(
    const std::shared_ptr<ControlTask::Request> request,
    std::shared_ptr<ControlTask::Response> response);

  /// 取一份一致的关节状态快照（链的顺序）；失败时给出中文原因。
  bool snapshotState(KDL::JntArray & q, KDL::JntArray & qdot, std::string & message);

  /// service 请求 → TaskRequest（只做"方言翻译"，合法性交给 TaskRouter 校验）。
  bool buildTaskRequest(
    const ControlTask::Request & request, kdl_control::TaskRequest & task,
    std::string & message);

  /// ControlResult（连续轨迹）→ JointTrajectory（含逐点逆动力学前馈 τ_ff）。
  bool buildTrajectory(
    const kdl_control::ControlResult & result,
    trajectory_msgs::msg::JointTrajectory & trajectory, std::string & message);

  /// 等控制器把轨迹接手（active:true），最多 timeout 秒。
  bool waitForActive(double timeout);
  /// 等控制器跑完（active:false），最多 timeout 秒。
  bool waitForInactive(double timeout);

  static void fillError(
    std::shared_ptr<ControlTask::Response> response, int32_t code, const std::string & message);

  // ---- 配置（构造期确定，之后只读）----
  std::string urdf_file_{};
  std::string reference_topic_{};
  std::string status_topic_{};

  /// 控制器侧（= JointTrajectory 消息里）的关节顺序。
  std::vector<std::string> controller_joints_{};

  /// KDL 链顺序的关节名（= TaskRouter 输出顺序 = 状态快照顺序）。
  std::vector<std::string> chain_joints_{};

  /// controller_joints_[i] 在链里的下标。
  std::vector<unsigned int> chain_index_of_controller_joint_{};

  double point_dt_ = 0.002;
  double result_timeout_margin_ = 5.0;
  double min_duration_ = 1.0;
  double tracking_tolerance_ = 0.05;
  std::size_t max_trajectory_points_ = kDefaultMaxTrajectoryPoints;

  // ---- 运行期状态 ----
  kdl_control::TaskRouter router_{};

  /// /joint_states 里最近一帧的状态，键是关节名。
  std::unordered_map<std::string, std::pair<double, double>> state_{};
  bool state_ready_ = false;
  std::mutex state_mutex_;

  /// 控制器状态回报（最近一帧），供 waitFor* 使用。
  std::mutex status_mutex_;
  std::condition_variable status_cv_;
  bool status_active_ = false;
  int32_t status_error_code_ = 0;
  std::string status_message_{};

  /// 同一时刻只允许一条任务在飞。
  std::mutex task_mutex_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;

  rclcpp::Service<ControlTask>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<ControlStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_pub_;
};

// ===========================================================================
// 装配
// ===========================================================================

ControlNode::ControlNode()
: Node("control_node")
{
  std::string message;

  if (!declareAndReadParameters(message)) {
    throw std::runtime_error(message);
  }
  if (!setupContext(message)) {
    throw std::runtime_error(message);
  }
  if (!setupJointMapping(message)) {
    throw std::runtime_error(message);
  }

  // ---- 回调组 ----
  // service 回调会阻塞几秒（等轨迹跑完），必须放进可重入组并配合多线程执行器；
  // /joint_states 与 /status 各用独立组，保证它们永远不会被 service 挡住。
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ---- 订阅 /joint_states ----
  // QoS 用 SensorDataQoS（best effort）：它对"可靠发布者"（joint_state_broadcaster
  // 用 SystemDefaultsQoS，即 reliable）依然兼容，反过来则不一定。
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = state_cb_group_;
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&ControlNode::jointStatesCallback, this, std::placeholders::_1),
    sub_options);

  // ---- 订阅控制器状态 ----
  status_sub_ = create_subscription<ControlStatus>(
    status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&ControlNode::statusCallback, this, std::placeholders::_1),
    sub_options);

  // ---- 参考轨迹发布 ----
  reference_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    reference_topic_, rclcpp::QoS(1).reliable());

  // ---- service ----
  service_ = create_service<ControlTask>(
    "/control_task",
    std::bind(
      &ControlNode::controlTaskCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_cb_group_);

  // 启动横幅：把"这一刻实际生效的配置"打出来。
  std::string mapping;
  for (std::size_t i = 0; i < controller_joints_.size(); ++i) {
    mapping += controller_joints_[i] + "(链#" +
               std::to_string(chain_index_of_controller_joint_[i]) + ")";
    if (i + 1 < controller_joints_.size()) { mapping += ", "; }
  }
  RCLCPP_INFO(
    get_logger(),
    "控制节点就绪：service=/control_task，reference=%s，status=%s\n"
    "  URDF      : %s\n"
    "  轨迹采样  : point_dt=%.4f s（%.0f 个点/秒，上限 %zu 点）\n"
    "  自动定时  : duration<=0 时由底层估算，但不短于 min_duration=%.3f s\n"
    "  关节映射  : %s",
    reference_topic_.c_str(), status_topic_.c_str(), urdf_file_.c_str(), point_dt_,
    1.0 / point_dt_, max_trajectory_points_, min_duration_, mapping.c_str());
}

bool ControlNode::declareAndReadParameters(std::string & message)
{
  // URDF 用**纯模型**（src/model/robotic_arm.urdf）：只描述运动学/惯量/行程，
  // 没有 <ros2_control> 标签，正好是 TaskRouter 与前馈计算需要的全部信息。
  urdf_file_ = declare_parameter<std::string>("urdf_file", "");
  if (urdf_file_.empty()) {
    try {
      urdf_file_ = ament_index_cpp::get_package_share_directory("kdl_tools") +
                   "/model/robotic_arm.urdf";
    } catch (const std::exception & e) {
      message = std::string("找不到 kdl_tools 的 share 目录（colcon build 并 source 了吗？）: ") +
                e.what();
      return false;
    }
  }

  reference_topic_ = declare_parameter<std::string>("reference_topic", "/control_reference");
  status_topic_ = declare_parameter<std::string>(
    "status_topic", "/kdl_effort_controller/status");

  // 关节顺序必须与 controllers.yaml 里 kdl_effort_controller.joints 一致 ——
  // 那是 JointTrajectory 消息的语义顺序（顺序不对外暴露在消息里，只能约定）。
  controller_joints_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {"joint1", "joint2", "joint3", "joint4"});

  point_dt_ = declare_parameter<double>("point_dt", 0.002);
  result_timeout_margin_ = declare_parameter<double>("result_timeout_margin", 5.0);
  min_duration_ = declare_parameter<double>("min_duration", 1.0);
  // 终点到位自检阈值 [rad]：控制器"跑完时间"不等于"到位"，见 controlTaskCallback 尾部。
  tracking_tolerance_ = declare_parameter<double>("tracking_tolerance", 0.05);
  const int max_points = declare_parameter<int>(
    "max_trajectory_points", static_cast<int>(kDefaultMaxTrajectoryPoints));

  if (!(point_dt_ > 0.0) || !std::isfinite(point_dt_)) {
    message = "point_dt 必须是正数（当前 " + std::to_string(point_dt_) + "）";
    return false;
  }
  if (result_timeout_margin_ < 0.0) {
    message = "result_timeout_margin 不能为负";
    return false;
  }
  if (min_duration_ < 0.0) {
    message = "min_duration 不能为负（0 表示完全按底层自动估算）";
    return false;
  }
  if (tracking_tolerance_ < 0.0) {
    message = "tracking_tolerance 不能为负";
    return false;
  }
  if (max_points <= 0) {
    message = "max_trajectory_points 必须为正";
    return false;
  }
  max_trajectory_points_ = static_cast<std::size_t>(max_points);
  if (controller_joints_.empty()) {
    message = "joint_names 不能为空";
    return false;
  }
  if (reference_topic_.empty() || status_topic_.empty()) {
    message = "reference_topic / status_topic 不能为空";
    return false;
  }
  return true;
}

bool ControlNode::setupContext(std::string & message)
{
  kdl_control::RobotContext ctx;
  if (!kdl_control::TaskRouter::loadContextFromUrdf(urdf_file_, ctx, message)) {
    return false;
  }

  const unsigned int n = ctx.chain.getNrOfJoints();
  if (n == 0) {
    message = "URDF 截出来的链里没有可动关节: " + urdf_file_;
    return false;
  }

  // 速度/加速度/jerk 上限：URDF 里只有位置行程与力矩上限，这三项是"使用场景"的信息，
  // 必须在这里补上（否则自动定时会直接报"无法自动定时"）。默认值与旧桥一致。
  //
  // 加速度这里原来有个 `(n == 6) ? {30,30,30,80,80,80} : vector(n, 30.0)` 的特例，
  // 意思是"给 6 轴臂的腕部关节 80 rad/s²"。4 轴臂（n = 4）走不到那个分支，于是
  // joint4 实际拿到的是 30 —— 特例没生效，也没人发现。与其留一个"看起来在照顾腕部、
  // 其实永远不生效"的分支（换模型时还会误导人），不如去掉：四个关节统一 30。
  // 真要让腕部更快，就显式写出关节名/索引，别靠关节数去猜。
  const std::vector<double> default_velocity(n, 1.0);                       // rad/s
  const std::vector<double> default_acceleration(n, 30.0);                  // rad/s²
  const std::vector<double> default_jerk(n, 100.0);                          // rad/s³

  const auto velocity = declare_parameter<std::vector<double>>("max_velocity", default_velocity);
  const auto acceleration =
    declare_parameter<std::vector<double>>("max_acceleration", default_acceleration);
  const auto jerk = declare_parameter<std::vector<double>>("max_jerk", default_jerk);

  if (!fillLimits(velocity, n, ctx.joint_limits.max_velocity, "max_velocity", message) ||
    !fillLimits(acceleration, n, ctx.joint_limits.max_acceleration, "max_acceleration", message) ||
    !fillLimits(jerk, n, ctx.joint_limits.max_jerk, "max_jerk", message))
  {
    return false;
  }

  // 笛卡尔限位只影响 task2 的可行性校验与自动定时；这里取保守值，够用即可。
  ctx.cartesian_limits.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 0.5);
  ctx.cartesian_limits.max_linear_acceleration =
    declare_parameter<double>("max_linear_acceleration", 1.0);
  ctx.cartesian_limits.max_angular_velocity =
    declare_parameter<double>("max_angular_velocity", 1.0);
  ctx.cartesian_limits.max_angular_acceleration =
    declare_parameter<double>("max_angular_acceleration", 2.0);

  if (!ctx.valid()) {
    message = "RobotContext 装配结果不完整（行程数组长度与关节数不一致？）";
    return false;
  }
  router_.setContext(ctx);

  RCLCPP_INFO(
    get_logger(), "RobotContext 装配完成：%u 个关节，URDF 限位 + 参数限位都已就位", n);
  return true;
}

bool ControlNode::setupJointMapping(std::string & message)
{
  const auto & chain = router_.context().chain;

  // 关节顺序要从**段**里取，而且要跳过没有关节的段（固定连接）。
  // 用 getSegment(i).getJoint()、i 从 0 数到 getNrOfJoints()-1 是错的：
  // 一旦链里有固定段，关节数 != 段数，整个映射会错位一格。
  for (unsigned int seg = 0; seg < chain.getNrOfSegments(); ++seg) {
    const KDL::Joint & joint = chain.getSegment(seg).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    chain_joints_.emplace_back(joint.getName());
  }

  const unsigned int n_chain = static_cast<unsigned int>(chain_joints_.size());
  if (n_chain != chain.getNrOfJoints()) {
    message = "链里关节名收集数目与 getNrOfJoints() 不一致";
    return false;
  }

  // 控制器关节必须正好是链上关节的一个排列：少了 → 有关节没人执行；
  // 多了 → 控制器报 INVALID_JOINTS。两种情况都在启动时报出来。
  chain_index_of_controller_joint_.clear();
  for (const auto & name : controller_joints_) {
    const auto it = std::find(chain_joints_.begin(), chain_joints_.end(), name);
    if (it == chain_joints_.end()) {
      std::string listed;
      for (std::size_t i = 0; i < chain_joints_.size(); ++i) {
        listed += chain_joints_[i];
        if (i + 1 < chain_joints_.size()) { listed += ", "; }
      }
      message = "joint_names 里的 '" + name + "' 不在 KDL 链上（链上关节：" + listed + "）";
      return false;
    }
    chain_index_of_controller_joint_.push_back(
      static_cast<unsigned int>(std::distance(chain_joints_.begin(), it)));
  }
  if (controller_joints_.size() != chain_joints_.size()) {
    message = "控制器关节数 " + std::to_string(controller_joints_.size()) +
              " 与链上可动关节数 " + std::to_string(chain_joints_.size()) +
              " 不一致：Trajectory 里会漏关节或对不上控制器的关节表";
    return false;
  }
  return true;
}

// ===========================================================================
// 运行期
// ===========================================================================

void ControlNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // 只要有一帧缺关节 / 出现非有限值，就把 state_ready_ 置回 false：
  // 宁可让 service 回"状态未就绪"，也不要拿半截状态去解算。
  bool complete = true;
  for (const auto & name : chain_joints_) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      complete = false;
      break;
    }
    const std::size_t k = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (k >= msg->position.size()) {
      complete = false;
      break;
    }
    const double position = msg->position[k];
    const double velocity = (k < msg->velocity.size()) ? msg->velocity[k] : 0.0;
    if (!std::isfinite(position) || !std::isfinite(velocity)) {
      complete = false;
      break;
    }
    state_[name] = {position, velocity};
  }
  state_ready_ = complete;
  if (!complete) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "/joint_states 里缺少链上关节（或值非有限），暂时无法解算任务");
  }
}

void ControlNode::statusCallback(const ControlStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_active_ = msg->active;
  status_error_code_ = msg->error_code;
  status_message_ = msg->message;
  status_cv_.notify_all();
}

bool ControlNode::waitForActive(double timeout)
{
  std::unique_lock<std::mutex> lock(status_mutex_);
  return status_cv_.wait_for(
    lock, std::chrono::duration<double>(timeout), [this]() { return status_active_; });
}

bool ControlNode::waitForInactive(double timeout)
{
  std::unique_lock<std::mutex> lock(status_mutex_);
  return status_cv_.wait_for(
    lock, std::chrono::duration<double>(timeout), [this]() { return !status_active_; });
}

bool ControlNode::snapshotState(
  KDL::JntArray & q, KDL::JntArray & qdot, std::string & message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_ready_) {
    message = "还没收到完整的 /joint_states（joint_state_broadcaster 起了吗？）";
    return false;
  }
  const unsigned int n = static_cast<unsigned int>(chain_joints_.size());
  q.resize(n);
  qdot.resize(n);
  for (unsigned int i = 0; i < n; ++i) {
    const auto it = state_.find(chain_joints_[i]);
    q(i) = it->second.first;
    qdot(i) = it->second.second;
  }
  return true;
}

bool ControlNode::buildTaskRequest(
  const ControlTask::Request & request, kdl_control::TaskRequest & task, std::string & message)
{
  // 时间参数两条任务共用
  task.duration = request.duration;

  const unsigned int n = static_cast<unsigned int>(chain_joints_.size());

  switch (request.task_type) {
    case ControlTask::Request::JOINT_SPACE: {
      if (request.goal_joint.size() != n) {
        message = "goal_joint 长度 " + std::to_string(request.goal_joint.size()) +
                  " 与关节数 " + std::to_string(n) + " 不匹配";
        return false;
      }
      task.type = kdl_control::TaskType::kJointSpace;
      task.name = "joint_space_from_service";
      task.goal_joint.resize(n);
      for (unsigned int i = 0; i < n; ++i) {
        task.goal_joint(i) = request.goal_joint[i];
      }
      return true;
    }
    case ControlTask::Request::CARTESIAN_SPACE: {
      task.type = kdl_control::TaskType::kCartesianSpace;
      task.name = "cartesian_space_from_service";
      // 起点由任务自己用 FK(q_now) 求，保证与实际状态一致（has_start_pose 保持 false）。
      task.goal_pose = frameFromPose(request.goal_pose);
      return true;
    }
    default:
      message = "未知 task_type = " + std::to_string(request.task_type) +
                "（0 = JOINT_SPACE，1 = CARTESIAN_SPACE）";
      return false;
  }
}

bool ControlNode::buildTrajectory(
  const kdl_control::ControlResult & result,
  trajectory_msgs::msg::JointTrajectory & trajectory, std::string & message)
{
  const kdl_interpolation::QuinticTrajectory & traj = result.joint_trajectory;
  const double duration = traj.duration();
  if (!traj.valid() || !(duration > 0.0) || !std::isfinite(duration)) {
    message = "解算结果里的轨迹无效（duration = " + std::to_string(duration) + "）";
    return false;
  }

  // k 从 0 到 ceil(duration/point_dt)：末点正好落在 t = duration 上（不是四舍五入
  // 后的近似值），且严格递增。
  const std::size_t num_points =
    static_cast<std::size_t>(std::ceil(duration / point_dt_)) + 1;
  if (num_points > max_trajectory_points_) {
    message = "轨迹要采样成 " + std::to_string(num_points) + " 个点，超过上限 " +
              std::to_string(max_trajectory_points_) +
              "：要么把 duration 调小，要么把 point_dt 调大";
    return false;
  }

  const unsigned int n_ctrl = static_cast<unsigned int>(controller_joints_.size());

  // header.stamp 留 0（不填）：语义 = "从现在开始执行"。
  trajectory.joint_names = controller_joints_;
  trajectory.points.clear();
  trajectory.points.reserve(num_points);

  KDL::JntArray q(traj.joints());
  KDL::JntArray qdot(traj.joints());
  KDL::JntArray qddot(traj.joints());

  for (std::size_t k = 0; k < num_points; ++k) {
    const double t = std::min(static_cast<double>(k) * point_dt_, duration);
    if (!traj.sample(t, q, qdot, qddot)) {
      message = "轨迹在 t = " + std::to_string(t) + " s 处采样失败";
      return false;
    }

    // 前馈力矩：τ_ff = M(q)·q̈ + C(q,q̇)·q̇ + G(q)。
    // 逆动力学返回的是**链顺序**的力矩，下面按名字映射到控制器顺序。
    const kdl_dynamics::IdResult id = kdl_dynamics::inverseDynamics(
      router_.context().chain, q, qdot, qddot);
    if (!id.success()) {
      message = "逆动力学在 t = " + std::to_string(t) + " s 处失败：" + id.message;
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(n_ctrl);
    point.velocities.resize(n_ctrl);
    point.effort.resize(n_ctrl);
    for (unsigned int i = 0; i < n_ctrl; ++i) {
      const unsigned int chain_index = chain_index_of_controller_joint_[i];
      point.positions[i] = q(chain_index);
      point.velocities[i] = qdot(chain_index);
      point.effort[i] = id.torque(chain_index);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory.points.push_back(point);
  }
  return true;
}

void ControlNode::controlTaskCallback(
  const std::shared_ptr<ControlTask::Request> request,
  std::shared_ptr<ControlTask::Response> response)
{
  // 一次只接一条任务：本节点会在下面阻塞着等轨迹跑完。
  std::unique_lock<std::mutex> busy_lock(task_mutex_, std::try_to_lock);
  if (!busy_lock.owns_lock()) {
    fillError(response, kBusy, "已有任务在执行（本期不支持排队/取消）");
    return;
  }

  // ---- 1. 当前关节状态 ----
  KDL::JntArray q_now;
  KDL::JntArray qdot_now;
  std::string message;
  if (!snapshotState(q_now, qdot_now, message)) {
    fillError(response, kNoJointState, message);
    return;
  }

  // ---- 2. 请求 → TaskRequest ----
  kdl_control::TaskRequest task;
  if (!buildTaskRequest(*request, task, message)) {
    fillError(response, kBadRequest, message);
    return;
  }

  // ---- 3. 解算（纯计算，不碰 ROS）----
  const bool auto_duration = !(task.duration > 0.0);

  kdl_control::ControlResult result = router_.dispatch(task, q_now, qdot_now);

  // 自动定时 + 下限重算（同旧桥）：底层的自动定时公式只保证速度/加速度上限，
  // 而五次插值的 jerk 是 ~Δ/T³ 量级，位移很小时 jerk 会直接超出上限。
  //
  // ⚠️ 这个重算**只在"自动定时给出的时长太短"时才有意义**：它把时长设成下限
  // min_duration_，也就是把 T 改小。如果第一次是**别的原因**失败（目标够不着、
  // IK 不收敛、限位超限……），重算只会让失败得更彻底，而且会把第一次那个更准确的
  // 原因盖掉。实测（4 轴臂，够不着的笛卡尔目标）：
  //     第一次   → "解析逆解失败：目标点超出工作空间"
  //     重算之后 → "第 0 段的线速度峰值 1.421345 m/s 超过上限 0.500000 m/s"
  // 调用者会顺着第二条去调速度限位，方向完全错了。所以**第一次的 message 必须留住**。
  if (auto_duration && min_duration_ > 0.0 &&
    (!result.success || result.duration < min_duration_))
  {
    const kdl_control::ControlResult first = result;
    RCLCPP_INFO(
      get_logger(), "%s，按下限 %.4f s 重算（第一次：%s）",
      first.success ? "自动定时结果低于下限" : "自动定时解算失败", min_duration_,
      first.success ? "成功" : first.message.c_str());
    task.duration = min_duration_;
    result = router_.dispatch(task, q_now, qdot_now);
    if (!result.success && !first.success) {
      result.message = first.message + "（另按 min_duration = " + std::to_string(min_duration_) +
                       " s 重算一次，仍失败：" + result.message + "）";
    }
  }

  response->trajectory_duration = result.duration;
  if (!result.success) {
    fillError(response, kSolveFailed, "解算失败：" + result.message);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "解算成功：%s，轨迹时长 %.4f s%s", kdl_control::taskTypeName(result.type),
    result.duration, auto_duration ? "（自动定时）" : "");

  // ---- 4. 连续轨迹 → 离散控制点（含前馈 τ_ff）----
  trajectory_msgs::msg::JointTrajectory trajectory;
  if (!buildTrajectory(result, trajectory, message)) {
    fillError(response, kSolveFailed, message);
    return;
  }

  // ---- 5. 发布参考轨迹 ----
  reference_pub_->publish(trajectory);
  RCLCPP_INFO(get_logger(), "已下发参考：%zu 个点，时长 %.4f s", trajectory.points.size(),
              result.duration);

  // ---- 6. 等控制器接手 → 跑完 ----
  // 控制器可能处于未激活 / 未加载状态：先等它把 active 置起来（2 s，够识破配置错误）。
  if (!waitForActive(2.0)) {
    fillError(
      response, kControllerNotActive,
      "控制器未接手轨迹（kdl_effort_controller 激活了吗？关节名匹配吗？）");
    return;
  }
  if (!waitForInactive(result.duration + result_timeout_margin_)) {
    fillError(
      response, kTimeout,
      "等待执行结果超时（轨迹 " + std::to_string(result.duration) + " s + 余量 " +
        std::to_string(result_timeout_margin_) + " s）");
    return;
  }

  // ---- 7. 回报控制器结果 ----
  int32_t code = 0;
  std::string status_message;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    code = status_error_code_;
    status_message = status_message_;
  }

  // ---- 8. 终点到位自检 ----
  // 控制器"把参考时间跑完"不等于"机械臂到位"：若末端/连杆发生自碰撞、或力矩被限幅，
  // 关节会卡在半路，而参考时间照样走完。这里在终点核对一次实际残差，避免"没到位却报成功"。
  // 给 0.5 s（墙钟）让末端落定再量，避免把正常收尾的暂态误判成超差。
  if (code == 0 && !trajectory.points.empty()) {
    // 先等机械臂"真的停下来"再量：轮询到各关节速度都足够小，或者最多等 settle_timeout。
    // 固定延时（例如 0.5 s）不够——低增益的手腕关节收尾很慢，会误判成超差。
    constexpr double kSettleTimeout = 3.0;   // s（墙钟）
    constexpr double kSettleVelocity = 0.05;  // rad/s
    const auto t_start = std::chrono::steady_clock::now();
    KDL::JntArray q_end;
    KDL::JntArray qdot_end;
    std::string check_message;
    while (true) {
      if (!snapshotState(q_end, qdot_end, check_message)) {
        break;
      }
      double v_max = 0.0;
      for (unsigned int i = 0; i < qdot_end.rows(); ++i) {
        v_max = std::max(v_max, std::fabs(qdot_end(i)));
      }
      const double waited =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
      if (v_max < kSettleVelocity || waited > kSettleTimeout) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (q_end.rows() > 0) {
      const auto & last = trajectory.points.back();
      double max_error = 0.0;
      unsigned int worst = 0;
      for (unsigned int i = 0; i < controller_joints_.size(); ++i) {
        const double error = std::fabs(
          last.positions[i] - q_end(chain_index_of_controller_joint_[i]));
        if (error > max_error) {
          max_error = error;
          worst = i;
        }
      }
      if (max_error > tracking_tolerance_) {
        fillError(
          response, kNotReached,
          "轨迹时间已跑完，但终点未到位：关节 " + controller_joints_[worst] + " 残差 " +
            std::to_string(max_error) + " rad（阈值 " + std::to_string(tracking_tolerance_) +
            "）。常见原因：末端/连杆自碰撞挡住了关节，或力矩被限幅");
        return;
      }
    }
  }

  response->error_code = code;
  response->success = (code == 0);
  std::string reply =
    response->success ?
    ("执行完成：轨迹 " + std::to_string(result.duration) + " s") :
    ("控制器返回 error_code = " + std::to_string(code) + "：" + status_message);
  if (response->success && result.pose_residual > 0.0) {
    // 4 轴臂够不着参考姿态里"工具轴指向"那一维，这个残差天然不为 0（见
    // ControlResult::pose_residual）。在这里如实报出来，免得调用者把"姿态跟不到
    // 参考值"当成控制器的问题 —— 那是构型能力，不是跟踪误差。
    reply += "；参考姿态与实到姿态的完整夹角 " + std::to_string(result.pose_residual) +
             " rad（4 轴臂够不着工具轴指向，非跟踪误差）";
  }
  response->message = reply;
}

void ControlNode::fillError(
  std::shared_ptr<ControlTask::Response> response, int32_t code, const std::string & message)
{
  response->success = false;
  response->error_code = code;
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ControlNode> node;
  try {
    node = std::make_shared<ControlNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("control_node"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // 必须是多线程：service 回调会阻塞着等 status，那条线程被占住的同时，
  // 还得有别的线程去收 /joint_states 与 /status。单线程会死锁。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
