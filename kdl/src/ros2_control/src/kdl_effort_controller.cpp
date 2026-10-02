// Copyright (c) 2026, kdl_tools authors.
// 教学用途：KdlEffortController 的实现（控制律与符号说明见头文件）。
//
// 一个周期（update）里做的事，顺序不能乱：
//   1) 看 RealtimeBuffer 有没有新参考轨迹 → 有就复位积分、置 active；
//   2) 从 state interface 读实测 q / q̇（与写命令同周期，无跨话题延迟）；
//   3) 算参考 (q_ref, v_ref, τ_ff)：执行中用轨迹点，空闲/结束用重力前馈；
//   4) τ = τ_ff + PID(Δq, Δq̇, ∫Δq)，双层限幅，写入 effort 命令接口；
//   5) 限频发布 ~/status。

#include "kdl_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <kdl/frames.hpp>

#include "kdl_idynamics.hpp"
#include "kdl_tools.hpp"
#include "kdl_tools/msg/control_status.hpp"

namespace kdl_tools
{

namespace
{

double clampValue(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

/// 读一个 state interface 的值。
/// Jazzy 起 get_value() 已废弃（Kilted 会移除），改用 get_optional()；
/// 拿不到锁时返回 NaN（上层不做额外处理，因为 500 Hz 下下一周期即可恢复）。
double readState(const hardware_interface::LoanedStateInterface & interface)
{
  const std::optional<double> value = interface.get_optional<double>();
  return value.has_value() ? value.value() : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

KdlEffortController::KdlEffortController() = default;

// ===========================================================================
// 生命周期
// ===========================================================================

controller_interface::CallbackReturn KdlEffortController::on_init()
{
  std::string message;
  if (!readParameters(message)) {
    RCLCPP_ERROR(get_node()->get_logger(), "参数错误：%s", message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
KdlEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
KdlEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::CallbackReturn KdlEffortController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string message;

  if (!setupModel(message)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!readGains(message)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 预分配：RT 线程内不再分配。
  q_meas_.assign(n_, 0.0);
  qd_meas_.assign(n_, 0.0);
  q_ref_.assign(n_, 0.0);
  v_ref_.assign(n_, 0.0);
  tau_ff_.assign(n_, 0.0);
  integral_.assign(n_, 0.0);
  hold_q_.assign(n_, 0.0);
  pos_idx_.assign(n_, -1);
  vel_idx_.assign(n_, -1);
  q_kdl_ = KDL::JntArray(n_);
  dq_zero_kdl_ = KDL::JntArray(n_);
  ddq_zero_kdl_ = KDL::JntArray(n_);
  for (unsigned int i = 0; i < n_; ++i) {
    dq_zero_kdl_(i) = 0.0;
    ddq_zero_kdl_(i) = 0.0;
  }

  // 参考轨迹订阅：回调在非 RT 线程，负责校验 + 规整 + 交给 RealtimeBuffer。
  trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    reference_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&KdlEffortController::trajectoryCallback, this, std::placeholders::_1));

  // 状态回报（~/status → /<controller_name>/status）。
  auto publisher = get_node()->create_publisher<kdl_tools::msg::ControlStatus>(
    "~/status", rclcpp::QoS(10).reliable());
  status_pub_ = std::make_shared<StatusPublisher>(publisher);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "已配置：%u 个关节，参考话题 %s，增益来自 %s\n"
    "  重力前馈：inverseDynamics(q,0,0)（已是补偿力矩，不取负）",
    n_, reference_topic_.c_str(), pid_config_file_.c_str());

  for (unsigned int i = 0; i < n_; ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(), "  %s: p=%.3f i=%.3f d=%.3f u=[%.3f,%.3f] i_clamp=[%.3f,%.3f]",
      joints_[i].c_str(), gains_[i].p, gains_[i].i, gains_[i].d, gains_[i].u_min,
      gains_[i].u_max, gains_[i].i_min, gains_[i].i_max);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KdlEffortController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 按名字解析 state interface 下标（不依赖框架给出的顺序）。
  std::fill(pos_idx_.begin(), pos_idx_.end(), -1);
  std::fill(vel_idx_.begin(), vel_idx_.end(), -1);
  for (std::size_t k = 0; k < state_interfaces_.size(); ++k) {
    const std::string & full = state_interfaces_[k].get_name();
    const std::size_t slash = full.rfind('/');
    if (slash == std::string::npos) {
      continue;
    }
    const std::string joint = full.substr(0, slash);
    const std::string iface = full.substr(slash + 1);
    for (unsigned int i = 0; i < n_; ++i) {
      if (joint != joints_[i]) {
        continue;
      }
      if (iface == "position") {
        pos_idx_[i] = static_cast<int>(k);
      } else if (iface == "velocity") {
        vel_idx_[i] = static_cast<int>(k);
      }
    }
  }
  for (unsigned int i = 0; i < n_; ++i) {
    if (pos_idx_[i] < 0 || vel_idx_[i] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "关节 %s 缺少 position/velocity 状态接口，无法闭环",
        joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // 激活即锁位：把保持参考设为当前实测位置，并（在 update 里）施加重力前馈，
  // 这样控制器一激活臂就不会因无重力补偿而下坠。
  for (unsigned int i = 0; i < n_; ++i) {
    hold_q_[i] = readState(state_interfaces_[pos_idx_[i]]);
    integral_[i] = 0.0;
  }
  active_ = false;
  prev_active_ = false;
  current_traj_.reset();
  elapsed_ = 0.0;
  seg_idx_ = 0;
  last_status_time_ = get_node()->now();
  RCLCPP_INFO(get_node()->get_logger(), "已激活：进入重力锁位");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KdlEffortController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  // 停用瞬间用重力补偿力矩锁住当前位置，避免命令接口残留"轨迹中的瞬时力矩"。
  if (command_interfaces_.size() >= n_ && pos_idx_.size() == n_ && n_ > 0 &&
    pos_idx_[0] >= 0)
  {
    for (unsigned int i = 0; i < n_; ++i) {
      q_kdl_(i) = readState(state_interfaces_[pos_idx_[i]]);
    }
    const kdl_dynamics::IdResult g =
      kdl_dynamics::inverseDynamics(chain_, q_kdl_, dq_zero_kdl_, ddq_zero_kdl_);
    if (g.success()) {
      for (unsigned int i = 0; i < n_; ++i) {
        (void)command_interfaces_[i].set_value(g.torque(i));
      }
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "已停用");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
// 主控制律
// ===========================================================================

controller_interface::return_type KdlEffortController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (n_ == 0 || command_interfaces_.size() < n_) {
    return controller_interface::return_type::OK;
  }
  const double dt = period.seconds();

  // ---- 1. 新参考轨迹？----
  auto incoming = *new_trajectory_.readFromRT();
  if (incoming && incoming.get() != current_traj_.get()) {
    current_traj_ = incoming;
    elapsed_ = 0.0;
    seg_idx_ = 0;
    std::fill(integral_.begin(), integral_.end(), 0.0);
    active_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(), "开始执行参考轨迹：%.3f s / %u 点",
      current_traj_->duration, current_traj_->samples);
  }

  // ---- 2. 实测状态（与写命令同周期）----
  for (unsigned int i = 0; i < n_; ++i) {
    q_meas_[i] = readState(state_interfaces_[pos_idx_[i]]);
    qd_meas_[i] = readState(state_interfaces_[vel_idx_[i]]);
    // 极少数情况（并发抢锁失败）会读到 NaN：保持上一周期的命令，绝不要把 NaN 写下去。
    if (!std::isfinite(q_meas_[i]) || !std::isfinite(qd_meas_[i])) {
      return controller_interface::return_type::OK;
    }
  }

  // ---- 3. 参考与前馈 ----
  double progress = 1.0;
  if (active_ && current_traj_) {
    const Trajectory & tr = *current_traj_;
    elapsed_ += dt;
    if (elapsed_ >= tr.duration) {
      const unsigned int last = tr.samples - 1;
      for (unsigned int i = 0; i < n_; ++i) {
        hold_q_[i] = tr.q[last * n_ + i];
      }
      active_ = false;
      progress = 1.0;
      RCLCPP_INFO(get_node()->get_logger(), "参考轨迹执行完毕，转入重力锁位");
    } else {
      // 等间隔采样，索引随 elapsed_ 单调前进（O(1) 摊销）。
      while (seg_idx_ + 1 < tr.samples && tr.t[seg_idx_ + 1] <= elapsed_) {
        ++seg_idx_;
      }
      const unsigned int k0 = seg_idx_;
      const unsigned int k1 = std::min(seg_idx_ + 1, tr.samples - 1);
      double a = 0.0;
      if (k1 > k0 && tr.t[k1] > tr.t[k0]) {
        a = (elapsed_ - tr.t[k0]) / (tr.t[k1] - tr.t[k0]);
      }
      for (unsigned int i = 0; i < n_; ++i) {
        const double q0 = tr.q[k0 * n_ + i];
        const double q1 = tr.q[k1 * n_ + i];
        const double v0 = tr.qv[k0 * n_ + i];
        const double v1 = tr.qv[k1 * n_ + i];
        const double f0 = tr.tau[k0 * n_ + i];
        const double f1 = tr.tau[k1 * n_ + i];
        q_ref_[i] = q0 + a * (q1 - q0);
        v_ref_[i] = v0 + a * (v1 - v0);
        tau_ff_[i] = feedforward_ ? (f0 + a * (f1 - f0)) : 0.0;
      }
      progress = tr.duration > 0.0 ? elapsed_ / tr.duration : 1.0;
    }
  }
  if (!active_) {
    // 空闲/结束：PD 锁位 + 重力前馈。
    // 重力补偿用**保持参考点 q_ref** 而不是实测 q 来算（setpoint gravity comp）：
    // 这样在 q = q_ref 处 τ_ff 恰好抵住重力、PD 无输出 → 稳态零静差；
    // 若用实测 q 算，PID 会停在"G(q) + kp·Δq = 0"的偏移点上（即稳态下沉）。
    // 注意：inverseDynamics(q,0,0) 返回的**就是**执行器抵住重力所需的力矩，
    //      直接用作 τ_ff，不要取负（凭一维摆判据实测确认，详见头文件注释）。
    for (unsigned int i = 0; i < n_; ++i) {
      q_ref_[i] = hold_q_[i];
      v_ref_[i] = 0.0;
      q_kdl_(i) = q_ref_[i];
    }
    const kdl_dynamics::IdResult g =
      kdl_dynamics::inverseDynamics(chain_, q_kdl_, dq_zero_kdl_, ddq_zero_kdl_);
    for (unsigned int i = 0; i < n_; ++i) {
      tau_ff_[i] = g.success() ? g.torque(i) : 0.0;
    }
  }

  // ---- 4. 控制律 ----
  for (unsigned int i = 0; i < n_; ++i) {
    const double dq = q_ref_[i] - q_meas_[i];
    const double dqd = v_ref_[i] - qd_meas_[i];
    integral_[i] += dq * dt;
    const double i_term = clampValue(gains_[i].i * integral_[i], gains_[i].i_min, gains_[i].i_max);
    double u = gains_[i].p * dq + gains_[i].d * dqd + i_term;
    u = clampValue(u, gains_[i].u_min, gains_[i].u_max);
    double tau = tau_ff_[i] + u;
    if (effort_limit_[i] > 0.0) {
      tau = clampValue(tau, -effort_limit_[i], effort_limit_[i]);
    }
    (void)command_interfaces_[i].set_value(tau);
  }

  // ---- 5. 状态回报（限频；active 跳变立即发）----
  if (status_pub_) {
    const bool transition = (active_ != prev_active_);
    const double interval = 1.0 / status_publish_rate_;
    if (transition || (time - last_status_time_).seconds() >= interval) {
      status_msg_.header.stamp = time;
      status_msg_.active = active_;
      status_msg_.error_code = 0;
      status_msg_.message = active_ ? std::string("executing") : std::string("holding");
      status_msg_.progress = progress;
      status_msg_.duration = current_traj_ ? current_traj_->duration : 0.0;
      status_pub_->try_publish(status_msg_);
      last_status_time_ = time;
      prev_active_ = active_;
    }
  }

  return controller_interface::return_type::OK;
}

// ===========================================================================
// 装配辅助
// ===========================================================================

bool KdlEffortController::readParameters(std::string & message)
{
  joints_ = auto_declare<std::vector<std::string>>("joints", {});
  if (joints_.empty()) {
    message = "joints 不能为空";
    return false;
  }

  reference_topic_ = auto_declare<std::string>("reference_topic", "/control_reference");
  urdf_file_ = auto_declare<std::string>("urdf_file", "");
  pid_config_file_ = auto_declare<std::string>("pid_config_file", "");
  feedforward_ = auto_declare<bool>("feedforward", true);
  status_publish_rate_ = auto_declare<double>("status_publish_rate", 100.0);

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
  if (pid_config_file_.empty()) {
    try {
      pid_config_file_ = ament_index_cpp::get_package_share_directory("kdl_tools") +
                         "/config/mujoco_pids.yaml";
    } catch (const std::exception & e) {
      message = std::string("找不到 kdl_tools 的 share 目录: ") + e.what();
      return false;
    }
  }
  if (!(status_publish_rate_ > 0.0) || !std::isfinite(status_publish_rate_)) {
    message = "status_publish_rate 必须是正数";
    return false;
  }
  if (reference_topic_.empty()) {
    message = "reference_topic 不能为空";
    return false;
  }
  return true;
}

bool KdlEffortController::setupModel(std::string & message)
{
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file_, chain_)) {
    message = "无法从 URDF 建链：" + urdf_file_;
    return false;
  }
  n_ = chain_.getNrOfJoints();
  if (n_ != joints_.size()) {
    message = "URDF 可动关节数 " + std::to_string(n_) + " 与 joints 参数 " +
              std::to_string(joints_.size()) + " 不一致";
    return false;
  }

  // 力矩上限取 URDF <limit effort>：与 MuJoCo <motor ctrlrange>、控制器输出限幅同源。
  urdf::Model model;
  if (!kdl_tools::loadUrdfModel(urdf_file_, model)) {
    message = "无法解析 URDF 以读取关节限位：" + urdf_file_;
    return false;
  }
  effort_limit_.assign(n_, 0.0);
  for (unsigned int i = 0; i < n_; ++i) {
    const auto joint = model.getJoint(joints_[i]);
    if (!joint || !joint->limits) {
      message = "关节 " + joints_[i] + " 在 URDF 里缺少 <limit>";
      return false;
    }
    effort_limit_[i] = joint->limits->effort;
    if (!(effort_limit_[i] > 0.0)) {
      message = "关节 " + joints_[i] + " 的 <limit effort> 非正";
      return false;
    }
  }
  return true;
}

bool KdlEffortController::readGains(std::string & message)
{
  gains_.assign(n_, PidGains{});

  YAML::Node root;
  try {
    root = YAML::LoadFile(pid_config_file_);
  } catch (const std::exception & e) {
    message = "读取 PID 文件失败：" + pid_config_file_ + "（" + e.what() + "）";
    return false;
  }

  // 文件形如：
  //   /**:
  //     ros__parameters:
  //       pid_gains:
  //         position:
  //           joint1: {p, i, d, u_clamp_max, u_clamp_min, i_clamp_max, i_clamp_min}
  // 先剥掉最外层通配节点名与 ros__parameters 包装。
  YAML::Node params = root;
  if (root.IsMap() && root.size() > 0) {
    YAML::Node first = root.begin()->second;
    if (first.IsMap() && first["ros__parameters"]) {
      params = first["ros__parameters"];
    } else if (first.IsMap()) {
      params = first;
    }
  }
  const YAML::Node position = params["pid_gains"]["position"];
  if (!position || !position.IsMap()) {
    message = pid_config_file_ + " 里找不到 pid_gains.position";
    return false;
  }

  for (unsigned int i = 0; i < n_; ++i) {
    const YAML::Node joint = position[joints_[i]];
    if (!joint) {
      message = "pid_gains.position 里缺少关节 " + joints_[i];
      return false;
    }
    PidGains g;
    g.p = joint["p"] ? joint["p"].as<double>() : 0.0;
    g.i = joint["i"] ? joint["i"].as<double>() : 0.0;
    g.d = joint["d"] ? joint["d"].as<double>() : 0.0;
    g.u_max = joint["u_clamp_max"] ? joint["u_clamp_max"].as<double>() : effort_limit_[i];
    g.u_min = joint["u_clamp_min"] ? joint["u_clamp_min"].as<double>() : -effort_limit_[i];
    g.i_max = joint["i_clamp_max"] ? joint["i_clamp_max"].as<double>() : 1e9;
    g.i_min = joint["i_clamp_min"] ? joint["i_clamp_min"].as<double>() : -1e9;
    gains_[i] = g;
  }
  return true;
}

// ===========================================================================
// 参考轨迹接收（非 RT 线程）
// ===========================================================================

void KdlEffortController::trajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || msg->points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "收到空轨迹，忽略");
    return;
  }
  const std::size_t m = msg->joint_names.size();
  if (m != n_) {
    RCLCPP_WARN(
      get_node()->get_logger(), "轨迹关节数 %zu 与控制器 %u 不一致，忽略", m, n_);
    return;
  }
  // 按名字把消息关节映射到控制器关节顺序。
  std::vector<int> map(m, -1);
  for (std::size_t k = 0; k < m; ++k) {
    for (unsigned int i = 0; i < n_; ++i) {
      if (msg->joint_names[k] == joints_[i]) {
        map[k] = static_cast<int>(i);
        break;
      }
    }
    if (map[k] < 0) {
      RCLCPP_WARN(
        get_node()->get_logger(), "轨迹里的关节 '%s' 不在控制器关节表，忽略",
        msg->joint_names[k].c_str());
      return;
    }
  }

  auto traj = std::make_shared<Trajectory>();
  traj->n = n_;
  traj->samples = static_cast<unsigned int>(msg->points.size());
  traj->t.resize(traj->samples);
  traj->q.assign(static_cast<std::size_t>(traj->samples) * n_, 0.0);
  traj->qv.assign(static_cast<std::size_t>(traj->samples) * n_, 0.0);
  traj->tau.assign(static_cast<std::size_t>(traj->samples) * n_, 0.0);

  for (unsigned int s = 0; s < traj->samples; ++s) {
    const auto & p = msg->points[s];
    if (p.positions.size() != m) {
      RCLCPP_WARN(get_node()->get_logger(), "第 %u 个路点 positions 长度不对，忽略", s);
      return;
    }
    const bool has_vel = (p.velocities.size() == m);
    // 注意：trajectory_msgs/JointTrajectoryPoint 里的字段名是**单数** effort（历史命名）。
    const bool has_tau = (p.effort.size() == m);
    traj->t[s] = rclcpp::Duration(p.time_from_start).seconds();
    for (std::size_t k = 0; k < m; ++k) {
      const std::size_t i = static_cast<std::size_t>(map[k]);
      traj->q[static_cast<std::size_t>(s) * n_ + i] = p.positions[k];
      traj->qv[static_cast<std::size_t>(s) * n_ + i] = has_vel ? p.velocities[k] : 0.0;
      traj->tau[static_cast<std::size_t>(s) * n_ + i] = has_tau ? p.effort[k] : 0.0;
    }
  }
  traj->duration = traj->t.empty() ? 0.0 : traj->t.back();
  if (!(traj->duration > 0.0)) {
    RCLCPP_WARN(get_node()->get_logger(), "轨迹总时长非正，忽略");
    return;
  }

  new_trajectory_.writeFromNonRT(std::shared_ptr<const Trajectory>(traj));
  RCLCPP_INFO(
    get_node()->get_logger(), "收到参考轨迹：%u 点，%.3f s", traj->samples, traj->duration);
}

}  // namespace kdl_tools

PLUGINLIB_EXPORT_CLASS(kdl_tools::KdlEffortController, controller_interface::ControllerInterface)
