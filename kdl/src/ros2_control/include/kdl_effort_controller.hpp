// Copyright (c) 2026, kdl_tools authors.
// 教学用途：力矩控制器——把 /control_reference 上的关节控制点变成关节力矩。
//
// ===========================================================================
// 为什么必须做成 ros2_control 控制器插件（而不是独立节点）
// ===========================================================================
// 力矩控制是闭环：τ 依赖"当前时刻的实测 q/」，而这个反馈必须在与写命令**同一个
// 控制周期内**取到。若用独立 ROS 节点，反馈只能绕 /joint_states（本工程 50 Hz +
// DDS 延迟），对 500 Hz 的力矩环相当于塞进一个大延迟，必然振荡。
//
// 作为控制器插件，本类的 update() 由 controller_manager 在 500 Hz 的
// read→update→write 环里同步调用：读 state interface、算 τ、写 effort 接口，
// 三者同周期完成，没有跨进程/跨话题的时序问题。
//
// ===========================================================================
// 控制律（与 PLAN_TORQUE.md §6 一致）
// ===========================================================================
//   Δq = q_ref − q_meas        Δq̇ = q̇_ref − q̇_meas
//   I  += Δq · period
//   τ_pid = kp·Δq + kd·Δq̇ + ki·I        （先对 ki·I 与 τ_pid 做 yaml 的双层限幅）
//   τ = clamp(τ_ff + τ_pid, ±URDF effort)
//
//   τ_ff 来源：
//     * 执行中：参考点里的 effort 字段（由 kdl_control_node 用逆动力学算好）；
//     * 空闲/结束后：inverseDynamics(chain, q_meas, 0, 0) —— 即重力补偿力矩。
//
// **关于重力前馈的符号（踩过一次，写死在这里）**：
//   本工程的 kdl_dynamics::inverseDynamics(q,0,0) 返回的就是"执行器为抵住重力所需
//   施加的力矩"，**直接用作 ctrl，不要取负**。已用一维摆判据实测确认：
//     ID(q,0,0) = -Lmg（= 抵住重力所需），FD(τ=-Lmg) → q̈=0（静止），
//     FD(τ=+Lmg) → q̈=+2Lmg（把重力加倍，臂会砸下来）。
//   （历史上 kdl_dynparam.hpp 的注释与 dynamics_demo 第 7 步曾写反成 "-G(q)"，
//     已在本次改造中一并修正；本控制器统一用 inverseDynamics，不用 gravityTorque。）
// ===========================================================================

#ifndef KDL_TOOLS__KDL_EFFORT_CONTROLLER_HPP_
#define KDL_TOOLS__KDL_EFFORT_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "kdl_tools/msg/control_status.hpp"

namespace kdl_tools
{

/**
 * @brief 前馈 + PID 反馈的关节力矩控制器（对应 MuJoCo 的 <motor>）。
 */
class KdlEffortController : public controller_interface::ControllerInterface
{
public:
  KdlEffortController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 一路 PID 增益（语义与 control_toolbox 一致：u_clamp 限总输出，i_clamp 限积分项）。
  struct PidGains
  {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double u_min = 0.0;
    double u_max = 0.0;
    double i_min = 0.0;
    double i_max = 0.0;
  };

  /// 规整后的参考轨迹（控制器关节顺序的扁平数组，供 RT 线程零分配索引）。
  struct Trajectory
  {
    unsigned int n = 0;        ///< 关节数
    unsigned int samples = 0;  ///< 采样点数
    double duration = 0.0;     ///< 总时长 [s]
    std::vector<double> t;     ///< 采样时刻 [s]，长度 = samples
    std::vector<double> q;     ///< 参考位置 [rad]，长度 = samples * n
    std::vector<double> qv;    ///< 参考速度 [rad/s]
    std::vector<double> tau;   ///< 前馈力矩 [N·m]
  };

  /// 读参数（joints_ / 话题 / 文件路径 / 开关）。
  bool readParameters(std::string & message);

  /// 从 URDF 建链并读各关节 effort 上限。
  bool setupModel(std::string & message);

  /// 从 mujoco_pids.yaml 读 pid_gains.position.<joint>。
  bool readGains(std::string & message);

  /// 订阅回调（非 RT 线程）：校验 + 规整 + 交给 RealtimeBuffer。
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  /// 发布一次状态（RT 线程，尽力而为，不阻塞）。
  void publishStatus(bool active, int32_t error_code, const std::string & message, double progress);

  // ---- 配置 ----
  std::vector<std::string> joints_{};
  std::string reference_topic_{};
  std::string urdf_file_{};
  std::string pid_config_file_{};
  bool feedforward_ = true;
  double status_publish_rate_ = 100.0;

  // ---- 模型 ----
  KDL::Chain chain_{};
  unsigned int n_ = 0;
  std::vector<double> effort_limit_{};  ///< URDF <limit effort>，力矩上限
  std::vector<PidGains> gains_{};

  /// state interface 在 state_interfaces_ 里的下标（on_activate 时按名字解析）。
  std::vector<int> pos_idx_{};
  std::vector<int> vel_idx_{};

  // ---- RT 状态 ----
  std::shared_ptr<const Trajectory> current_traj_{};
  double elapsed_ = 0.0;
  unsigned int seg_idx_ = 0;
  bool active_ = false;
  std::vector<double> integral_{};
  std::vector<double> hold_q_{};

  // 预分配缓冲：RT 线程内不做任何堆分配。
  std::vector<double> q_meas_{};
  std::vector<double> qd_meas_{};
  std::vector<double> q_ref_{};
  std::vector<double> v_ref_{};
  std::vector<double> tau_ff_{};
  KDL::JntArray q_kdl_{};
  KDL::JntArray dq_zero_kdl_{};
  KDL::JntArray ddq_zero_kdl_{};
  kdl_tools::msg::ControlStatus status_msg_{};

  realtime_tools::RealtimeBuffer<std::shared_ptr<const Trajectory>> new_trajectory_{};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;

  using StatusPublisher = realtime_tools::RealtimePublisher<kdl_tools::msg::ControlStatus>;
  std::shared_ptr<StatusPublisher> status_pub_{};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  bool prev_active_ = false;
};

}  // namespace kdl_tools

#endif  // KDL_TOOLS__KDL_EFFORT_CONTROLLER_HPP_
