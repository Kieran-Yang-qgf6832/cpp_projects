// Copyright (c) 2026, kdl_identification authors.
// 教学用途：参数辨识的"数据采集 + 求解"节点——把前两块（激励轨迹、回归矩阵+最小二乘）
// 接到真实的 MuJoCo 闭环里跑一遍。
//
// ===========================================================================
// 链路
// ===========================================================================
//   kdl_identify_node
//     ├─ ① 优化一条傅里叶激励轨迹（kdl_fourier）
//     ├─ ② 按 point_dt 采样成 JointTrajectory（position/velocity/effort）
//     │      effort 填 inverseDynamics(名义模型, q_ref, v_ref, a_ref)：
//     │      控制器（feedforward: true，默认）拿它做前馈，臂才会"贴着参考动"；
//     │      这只是把一部分施力交给模型算，**不影响**辨识——辨识用的是实测 τ。
//     ├─ ③ publish /control_reference
//     ├─ ④ 订阅 /kdl_effort_controller/status 判断开始(active:true)/结束(active:false)
//     ├─ ⑤ 订阅 /mujoco_actuators_states 收 (q, q̇, τ=qfrc_actuator)（500 Hz，同源同步）
//     ├─ ⑥ 只在测量窗内取点；q̈ 用 q̇ 的中心差分；拼回归矩阵（可选加摩擦列）
//     └─ ⑦ solveLeastSquares → 打印 + 存 CSV
//
// ===========================================================================
// 三个容易踩的点（都写死在代码里）
// ===========================================================================
//   1) **施力的是控制器，不是本节点**：本节点只把参考轨迹和"模型前馈"发过去，
//      真正闭环在 500 Hz 的控制周期内。所以 τ 的实测值必须从
//      /mujoco_actuators_states 的 effort 字段取（= qfrc_actuator）。
//   2) **必须绕开第一段暂态**：控制器接手后有一段过渡。用 warmup_periods 丢掉
//      前若干个周期，只取 measure_periods 个周期做辨识。
//   3) **MuJoCo 的关节阻尼不在 qfrc_actuator 里**：阻尼走的是 qfrc_passive，
//      所以实测 τ 满足 τ = Y·β + diag(d)·q̇。默认打开 use_friction_model，
//      多辨识 n 个粘性 + n 个库伦系数，正好能把 MJCF 里的关节阻尼找回来。
//      （注意：当前 4 轴模型的 URDF 没有 <dynamics>、MJCF 也未设关节阻尼，
//        所以这两项会辨识到 ≈0 —— 这是符合预期的，不是辨识失败。）
// ===========================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>

#include "kdl_fourier.hpp"
#include "kdl_idynamics.hpp"
#include "kdl_lsq.hpp"
#include "kdl_quintic.hpp"
#include "kdl_regressor.hpp"
#include "kdl_tools.hpp"
#include "kdl_tools/msg/control_status.hpp"

namespace
{

/// 默认输出目录（源码树的 draw 文件夹）：由 CMake 在编译期传入。
/// 这样辨识结果与后面的图片都集中落在同一个方便查找的位置。
#ifndef KDL_IDENTIFY_DRAW_DIR
#define KDL_IDENTIFY_DRAW_DIR "."
#endif

using kdl_tools::msg::ControlStatus;

/// 一个采样点的原始记录（时间 + 实测 q / q̇ / τ），都是链顺序。
struct Record
{
  double t = 0.0;
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};

/**
 * @brief 对一整条扁平数组做中心滑动平均（边界保持原值）。
 * @param values [in] 长度 = N × n 的扁平数组。
 * @param window [in] 窗口长度（奇数更对称）；<= 1 表示不平滑。
 * @return 平滑后的数组。
 */
std::vector<double> movingAverage(const std::vector<double> & values, int window)
{
  const int size = static_cast<int>(values.size());
  if (window <= 1 || size == 0) {
    return values;
  }
  const int half = window / 2;
  std::vector<double> out(values.size(), 0.0);
  for (int i = 0; i < size; ++i) {
    const int lo = std::max(0, i - half);
    const int hi = std::min(size - 1, i + half);
    double sum = 0.0;
    for (int k = lo; k <= hi; ++k) {
      sum += values[k];
    }
    out[i] = sum / (hi - lo + 1);
  }
  return out;
}

}  // namespace

namespace kdl_identification
{

/**
 * @brief 辨识节点：跑一次激励实验，采集数据，解出参数。
 */
class IdentifyNode : public rclcpp::Node
{
public:
  IdentifyNode();
  ~IdentifyNode() override = default;

private:
  /// 节点状态机。
  enum class State
  {
    kWaiting,     ///< 等仿真/控制器就绪（收到过 status 与 actuator 状态）
    kIdle,        ///< 就绪，等待触发（自动或 service）
    kExciting,    ///< 激励轨迹已下发，正在采集
    kDone,        ///< 本轮完成
  };

  // ---- 初始化 ----
  void readParameters();
  bool setupModel(std::string & message);
  void setupRos();

  // ---- 触发与流程 ----
  void onRunService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool startExperiment(std::string & message);
  bool buildAndPublishTrajectory(
    const FourierTrajectory & trajectory, double total_duration, std::string & message);

  // ---- 回调 ----
  void onActuatorState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onStatus(const ControlStatus::SharedPtr msg);
  void onTimer();

  // ---- 收尾 ----
  void finalize();
  bool solveAndReport(const std::vector<Record> & window, const std::vector<double> & qddot,
    std::string & message);
  bool writeSamplesCsv(const std::vector<Record> & window, const std::vector<double> & qddot,
    const Eigen::VectorXd & prediction, const std::string & path) const;

  // ---- 配置 ----
  std::string urdf_file_{};
  std::string reference_topic_{};
  std::string status_topic_{};
  std::string actuator_topic_{};
  std::vector<std::string> joint_names_{};
  double point_dt_ = 0.002;
  double lead_in_time_ = 2.0;
  double warmup_periods_ = 1.0;
  double measure_periods_ = 2.0;
  double acceleration_over_velocity_ = 5.0;
  bool use_friction_model_ = true;
  int smoothing_window_ = 1;
  double rank_tolerance_ = 1e-9;
  bool auto_start_ = true;
  double start_delay_ = 2.0;
  std::string samples_csv_ = "identification_samples.csv";
  std::string parameters_csv_ = "identification_params.csv";
  ExcitationOptions excitation_options_{};

  // ---- 模型 ----
  KDL::Chain chain_{};
  unsigned int n_ = 0;
  FourierLimits limits_{};
  // RegressorBuilder 内部持有对链的引用、禁止搬移，所以用指针持有。
  std::unique_ptr<RegressorBuilder> builder_{};

  // ---- ROS ----
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr actuator_sub_;
  rclcpp::Subscription<ControlStatus>::SharedPtr status_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 运行状态 ----
  State state_ = State::kWaiting;
  bool have_status_ = false;
  bool have_actuator_ = false;
  bool mapping_ready_ = false;
  bool mapping_failed_ = false;
  bool auto_started_ = false;
  bool t_active_valid_ = false;
  bool finalized_ = false;
  bool latest_state_valid_ = false;
  double t_active_ = 0.0;
  double expected_duration_ = 0.0;
  double period_ = 0.0;
  std::chrono::steady_clock::time_point publish_wall_{};
  std::chrono::steady_clock::time_point start_wall_{};
  std::vector<int> actuator_index_of_joint_{};  // 链关节 j → 消息数组下标
  std::vector<Record> records_;
  Eigen::VectorXd latest_q_;                    // 最近一次实测关节角（做入场过渡用）
  Eigen::VectorXd last_prediction_;             // 最近一次辨识的预测力矩（写 CSV 给画图用）
};

// ===========================================================================
// 初始化
// ===========================================================================

IdentifyNode::IdentifyNode()
: Node("kdl_identify")
{
  readParameters();

  std::string message;
  if (!setupModel(message)) {
    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
    throw std::runtime_error(message);
  }
  setupRos();
  start_wall_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    get_logger(),
    "辨识节点就绪：\n"
    "  URDF        : %s\n"
    "  关节        : %s\n"
    "  参考话题    : %s\n"
    "  控制器状态  : %s\n"
    "  执行器状态  : %s\n"
    "  激励        : %u 谐波 / 周期 %.2f s / 入场 %.2f s + warmup %.1f 周期 + 测量 %.1f 周期\n"
    "  摩擦模型    : %s\n"
    "  样本 CSV    : %s\n"
    "  参数 CSV    : %s\n"
    "  触发        : %s（service ~/run）",
    urdf_file_.c_str(), [this] { std::string s; for (const auto & j : joint_names_) { s += j + " "; } return s; }().c_str(),
    reference_topic_.c_str(), status_topic_.c_str(), actuator_topic_.c_str(),
    excitation_options_.harmonics, excitation_options_.period, lead_in_time_, warmup_periods_,
    measure_periods_, use_friction_model_ ? "开" : "关", samples_csv_.c_str(),
    parameters_csv_.c_str(), auto_start_ ? "自动" : "手动");
}

void IdentifyNode::readParameters()
{
  urdf_file_ = declare_parameter<std::string>("urdf_file", "");
  if (urdf_file_.empty()) {
    urdf_file_ = ament_index_cpp::get_package_share_directory("kdl_tools") + "/model/robotic_arm.urdf";
  }
  reference_topic_ = declare_parameter<std::string>("reference_topic", "/control_reference");
  status_topic_ =
    declare_parameter<std::string>("status_topic", "/kdl_effort_controller/status");
  actuator_topic_ =
    declare_parameter<std::string>("actuator_states_topic", "/mujoco_actuators_states");
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {"joint1", "joint2", "joint3", "joint4"});

  point_dt_ = declare_parameter<double>("point_dt", 0.002);
  lead_in_time_ = declare_parameter<double>("lead_in_time", 2.0);
  warmup_periods_ = declare_parameter<double>("warmup_periods", 1.0);
  measure_periods_ = declare_parameter<double>("measure_periods", 2.0);
  acceleration_over_velocity_ = declare_parameter<double>("acceleration_over_velocity", 5.0);
  use_friction_model_ = declare_parameter<bool>("use_friction_model", true);
  smoothing_window_ = declare_parameter<int>("smoothing_window", 1);
  rank_tolerance_ = declare_parameter<double>("rank_tolerance", 1e-9);
  auto_start_ = declare_parameter<bool>("auto_start", true);
  start_delay_ = declare_parameter<double>("start_delay", 2.0);
  // 样本/参数 CSV 默认写到源码树的 draw 目录（与画图脚本同一处），可用参数覆盖。
  const std::string default_dir = KDL_IDENTIFY_DRAW_DIR;
  samples_csv_ = declare_parameter<std::string>(
    "samples_csv", default_dir + "/identification_samples.csv");
  parameters_csv_ = declare_parameter<std::string>(
    "parameters_csv", default_dir + "/identification_params.csv");

  excitation_options_.harmonics =
    static_cast<unsigned int>(declare_parameter<int>("harmonics", 5));
  excitation_options_.period = declare_parameter<double>("period", 4.0);
  excitation_options_.samples_per_period =
    static_cast<unsigned int>(declare_parameter<int>("samples_per_period", 50));
  excitation_options_.restarts =
    static_cast<unsigned int>(declare_parameter<int>("restarts", 15));
  excitation_options_.refine_iterations =
    static_cast<unsigned int>(declare_parameter<int>("refine_iterations", 8));
  excitation_options_.max_evaluations =
    static_cast<unsigned int>(declare_parameter<int>("max_evaluations", 150));
  excitation_options_.rank_tolerance = rank_tolerance_;
  excitation_options_.random_seed =
    static_cast<unsigned int>(declare_parameter<int>("random_seed", 20261005));
}

bool IdentifyNode::setupModel(std::string & message)
{
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file_, chain_)) {
    message = "无法从 URDF 建链：" + urdf_file_;
    return false;
  }
  n_ = chain_.getNrOfJoints();
  if (n_ != joint_names_.size()) {
    message = "URDF 可动关节数 " + std::to_string(n_) + " 与 joint_names " +
      std::to_string(joint_names_.size()) + " 不一致";
    return false;
  }

  // 链顺序的关节名必须与 joint_names 一致（消息按名字对齐，但激励/回归都在链顺序上）。
  unsigned int index = 0;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    if (index >= joint_names_.size() || joint.getName() != joint_names_[index]) {
      message = "链上第 " + std::to_string(index) + " 个关节 '" + joint.getName() +
        "' 与 joint_names 对应项不一致";
      return false;
    }
    ++index;
  }

  // 关节限位：位置/速度来自 URDF，加速度取速度的固定倍数（URDF 里没有）。
  urdf::Model model;
  if (!kdl_tools::loadUrdfModel(urdf_file_, model)) {
    message = "无法解析 URDF 读取限位：" + urdf_file_;
    return false;
  }
  limits_.q_min = KDL::JntArray(n_);
  limits_.q_max = KDL::JntArray(n_);
  limits_.qdot_max = KDL::JntArray(n_);
  limits_.qddot_max = KDL::JntArray(n_);
  for (unsigned int j = 0; j < n_; ++j) {
    const auto joint = model.getJoint(joint_names_[j]);
    if (!joint || !joint->limits) {
      message = "关节 " + joint_names_[j] + " 缺少 <limit>";
      return false;
    }
    limits_.q_min(j) = joint->limits->lower;
    limits_.q_max(j) = joint->limits->upper;
    limits_.qdot_max(j) = joint->limits->velocity;
    limits_.qddot_max(j) = acceleration_over_velocity_ * joint->limits->velocity;
  }

  builder_ = std::make_unique<RegressorBuilder>(chain_);
  if (!builder_->valid()) {
    message = "回归矩阵构造器不可用：" + builder_->error();
    return false;
  }
  message.clear();
  return true;
}

void IdentifyNode::setupRos()
{
  reference_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    reference_topic_, rclcpp::QoS(1).reliable());
  // 仿真里状态是"尽力而为"就够：与 joint_state_broadcaster 的 reliable 兼容。
  actuator_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    actuator_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IdentifyNode::onActuatorState, this, std::placeholders::_1));
  status_sub_ = create_subscription<ControlStatus>(
    status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&IdentifyNode::onStatus, this, std::placeholders::_1));
  run_service_ = create_service<std_srvs::srv::Trigger>(
    "~/run", std::bind(&IdentifyNode::onRunService, this, std::placeholders::_1,
      std::placeholders::_2));
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&IdentifyNode::onTimer, this));
}

// ===========================================================================
// 触发与下发
// ===========================================================================

void IdentifyNode::onRunService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string message;
  if (startExperiment(message)) {
    response->success = true;
    response->message = message;
  } else {
    response->success = false;
    response->message = message;
  }
}

void IdentifyNode::onTimer()
{
  if (!have_status_ || !have_actuator_) {
    state_ = State::kWaiting;
    return;
  }
  if (state_ == State::kWaiting) {
    state_ = State::kIdle;
  }

  // 自动触发（延迟 start_delay 秒，等控制器锁稳）。
  if (state_ == State::kIdle && auto_start_ && !auto_started_) {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_wall_).count();
    if (elapsed >= start_delay_) {
      auto_started_ = true;
      std::string message;
      if (!startExperiment(message)) {
        RCLCPP_ERROR(get_logger(), "自动触发失败：%s", message.c_str());
      }
    }
  }

  // 超时保护（墙钟）：控制器没报结束也要收尾。
  if (state_ == State::kExciting && !finalized_) {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - publish_wall_).count();
    if (elapsed > expected_duration_ + 10.0) {
      RCLCPP_WARN(get_logger(), "等待控制器结束超时，按已采集数据收尾");
      finalize();
    }
  }
}

bool IdentifyNode::startExperiment(std::string & message)
{
  if (state_ == State::kExciting) {
    message = "上一轮激励还在进行中";
    return false;
  }
  if (!have_status_ || !have_actuator_ || !latest_state_valid_) {
    message = "仿真/控制器尚未就绪（没收到 status 或 actuator 状态）";
    return false;
  }

  RCLCPP_INFO(get_logger(), "开始优化激励轨迹（可能要几秒）……");
  const ExcitationResult excitation = optimizeFourierExcitation(chain_, limits_, excitation_options_);
  if (!excitation.success) {
    message = "激励轨迹优化失败：" + excitation.message;
    return false;
  }
  RCLCPP_INFO(
    get_logger(),
    "激励轨迹就绪：rank=%u/%u，σ_min⁺=%.3e，cond=%.3e，λ=%.4f",
    excitation.observability.rank, excitation.observability.total_parameters,
    excitation.observability.min_singular_value, excitation.observability.condition_number,
    excitation.scale);

  period_ = excitation.trajectory.period();
  const double fourier_duration = (warmup_periods_ + measure_periods_) * period_;
  if (!buildAndPublishTrajectory(excitation.trajectory, fourier_duration, message)) {
    return false;
  }

  records_.clear();
  records_.reserve(static_cast<std::size_t>(
    ((lead_in_time_ + fourier_duration) / point_dt_) + 8));
  finalized_ = false;
  t_active_valid_ = false;
  t_active_ = 0.0;
  expected_duration_ = lead_in_time_ + fourier_duration;
  publish_wall_ = std::chrono::steady_clock::now();
  state_ = State::kExciting;

  message = "激励轨迹已下发：入场 " + std::to_string(lead_in_time_) + " s + 激励 " +
    std::to_string(fourier_duration) + " s（warmup + 测量）";
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  return true;
}

bool IdentifyNode::buildAndPublishTrajectory(
  const FourierTrajectory & trajectory, double fourier_duration, std::string & message)
{
  if (point_dt_ <= 0.0) {
    message = "point_dt 必须为正";
    return false;
  }
  if (!latest_state_valid_) {
    message = "缺少当前关节角，无法生成入场过渡";
    return false;
  }

  // 傅里叶轨迹在 t=0 处的状态（入场段要接到这里，才能速度/加速度连续）。
  KDL::JntArray q_begin(n_);
  KDL::JntArray qdot_begin(n_);
  KDL::JntArray qddot_begin(n_);
  if (!trajectory.sample(0.0, q_begin, qdot_begin, qddot_begin)) {
    message = "傅里叶轨迹采样失败";
    return false;
  }

  // 入场段：每关节一条五次多项式，从"当前实测位形 + 静止"平滑接到傅里叶起点。
  // 不做这一步的话，控制器一接手就要用大误差把臂拽到轨迹起点，容易冲击/撞限位。
  const double lead = std::max(0.0, lead_in_time_);
  std::vector<kdl_interpolation::QuinticCoefficients> lead_segments(n_);
  for (unsigned int j = 0; j < n_; ++j) {
    if (lead > 0.0) {
      if (!kdl_interpolation::computeQuinticCoefficients(
            latest_q_(j), 0.0, 0.0, q_begin(j), qdot_begin(j), qddot_begin(j), lead,
            lead_segments[j])) {
        message = "入场过渡求解失败（lead_in_time 必须为正）";
        return false;
      }
    }
  }

  const double total_duration = lead + fourier_duration;
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = joint_names_;
  const std::size_t num_points =
    static_cast<std::size_t>(std::ceil(total_duration / point_dt_)) + 1;
  msg.points.reserve(num_points);

  for (std::size_t k = 0; k < num_points; ++k) {
    const double t = std::min(k * point_dt_, total_duration);
    KDL::JntArray q(n_);
    KDL::JntArray qdot(n_);
    KDL::JntArray qddot(n_);

    if (lead > 0.0 && t < lead) {
      const double tau = t / lead;
      for (unsigned int j = 0; j < n_; ++j) {
        kdl_interpolation::evaluateQuinticSegment(
          lead_segments[j], tau, q(j), qdot(j), qddot(j));
      }
    } else if (!trajectory.sample(t - lead, q, qdot, qddot)) {
      message = "轨迹采样失败";
      return false;
    }

    // 前馈力矩：名义模型逆动力学。控制器 feedforward: true 时用它，
    // 臂才会贴着参考动；这只把一部分施力交给模型算，不影响用实测 τ 辨识。
    const kdl_dynamics::IdResult ff = kdl_dynamics::inverseDynamics(chain_, q, qdot, qddot);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(n_);
    point.velocities.resize(n_);
    point.effort.resize(n_);
    for (unsigned int j = 0; j < n_; ++j) {
      point.positions[j] = q(j);
      point.velocities[j] = qdot(j);
      point.effort[j] = ff.success() ? ff.torque(j) : 0.0;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    msg.points.push_back(point);
  }

  // header.stamp 留 0：语义 = "从现在开始执行"（与 kdl_control_node 一致）。
  reference_pub_->publish(msg);
  message.clear();
  return true;
}

// ===========================================================================
// 采集
// ===========================================================================

void IdentifyNode::onActuatorState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  have_actuator_ = true;
  if (mapping_failed_) {
    return;
  }

  // 关节名映射只建一次（消息里 include 了 MJCF 的所有关节，按名字对齐）。
  if (!mapping_ready_) {
    actuator_index_of_joint_.assign(n_, -1);
    for (unsigned int j = 0; j < n_; ++j) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[j]);
      if (it == msg->name.end()) {
        mapping_failed_ = true;
        RCLCPP_ERROR(get_logger(), "执行器状态里找不到关节 %s", joint_names_[j].c_str());
        return;
      }
      actuator_index_of_joint_[j] = static_cast<int>(std::distance(msg->name.begin(), it));
    }
    latest_q_.resize(n_);
    mapping_ready_ = true;
  }

  const std::size_t fields = msg->name.size();
  if (msg->position.size() < fields || msg->velocity.size() < fields || msg->effort.size() < fields) {
    RCLCPP_WARN_ONCE(get_logger(), "执行器状态字段长度不足，跳过该消息");
    return;
  }

  // 无论是否在激励中，都更新"最近实测位形"（下一条激励的入场过渡要用）。
  for (unsigned int j = 0; j < n_; ++j) {
    latest_q_(j) = msg->position[actuator_index_of_joint_[j]];
  }
  latest_state_valid_ = true;

  if (state_ != State::kExciting) {
    return;
  }

  Record record;
  record.t = rclcpp::Time(msg->header.stamp).seconds();
  record.q.resize(n_);
  record.qdot.resize(n_);
  record.tau.resize(n_);
  for (unsigned int j = 0; j < n_; ++j) {
    const int k = actuator_index_of_joint_[j];
    record.q(j) = msg->position[k];
    record.qdot(j) = msg->velocity[k];
    record.tau(j) = msg->effort[k];
  }
  records_.push_back(std::move(record));
}

void IdentifyNode::onStatus(const ControlStatus::SharedPtr msg)
{
  have_status_ = true;
  if (state_ != State::kExciting) {
    return;
  }

  if (msg->active && !t_active_valid_) {
    t_active_ = rclcpp::Time(msg->header.stamp).seconds();
    t_active_valid_ = true;
    RCLCPP_INFO(get_logger(), "控制器已接手激励轨迹（t=%.3f），开始计时", t_active_);
  } else if (!msg->active && t_active_valid_ && !finalized_) {
    finalize();
  }
}

// ===========================================================================
// 收尾：整理测量窗 → 求解 → 报告
// ===========================================================================

void IdentifyNode::finalize()
{
  finalized_ = true;
  state_ = State::kDone;

  if (records_.empty()) {
    RCLCPP_ERROR(get_logger(), "没有采集到任何执行器状态，无法辨识");
    return;
  }
  // 控制器接手后先跑入场段，傅里叶激励从 t0 + lead_in 才开始。
  const double t0 = t_active_valid_ ? t_active_ : records_.front().t;
  const double t_fourier_start = t0 + lead_in_time_;
  const double window_begin = t_fourier_start + warmup_periods_ * period_;
  const double window_end = t_fourier_start + (warmup_periods_ + measure_periods_) * period_;

  std::vector<Record> window;
  window.reserve(records_.size());
  for (const Record & r : records_) {
    if (r.t >= window_begin && r.t <= window_end) {
      window.push_back(r);
    }
  }
  RCLCPP_INFO(
    get_logger(), "采集到 %zu 个状态；测量窗 [%.3f, %.3f] 内 %zu 个", records_.size(),
    window_begin, window_end, window.size());

  if (window.size() < 50) {
    RCLCPP_ERROR(get_logger(), "测量窗内样本太少（%zu），放弃辨识", window.size());
    return;
  }

  // q̈：q̇ 的中心差分（仿真里 q̇ 很干净；smoothing_window>1 时可先平滑）。
  std::vector<double> flat_qdot(static_cast<std::size_t>(window.size()) * n_);
  std::vector<double> flat_tau(static_cast<std::size_t>(window.size()) * n_);
  for (std::size_t i = 0; i < window.size(); ++i) {
    for (unsigned int j = 0; j < n_; ++j) {
      flat_qdot[i * n_ + j] = window[i].qdot(j);
      flat_tau[i * n_ + j] = window[i].tau(j);
    }
  }
  flat_qdot = movingAverage(flat_qdot, smoothing_window_);
  flat_tau = movingAverage(flat_tau, smoothing_window_);
  for (std::size_t i = 0; i < window.size(); ++i) {
    for (unsigned int j = 0; j < n_; ++j) {
      window[i].qdot(j) = flat_qdot[i * n_ + j];
      window[i].tau(j) = flat_tau[i * n_ + j];
    }
  }

  // 相邻样本间隔的"正常值"：取中位数。DDS 偶发重复/乱序会让某两个样本间隔
  // 接近 0，中心差分除以极小的 dt 会把 q̈ 放大成尖刺（实测残差 MAX 的元凶）。
  // 所以只保留左右间隔都接近中位数的点。
  std::vector<double> gaps;
  gaps.reserve(window.size());
  for (std::size_t i = 1; i < window.size(); ++i) {
    gaps.push_back(window[i].t - window[i - 1].t);
  }
  std::vector<double> sorted_gaps = gaps;
  std::sort(sorted_gaps.begin(), sorted_gaps.end());
  const double median_dt = sorted_gaps.empty() ? 0.0 : sorted_gaps[sorted_gaps.size() / 2];

  // 第一遍：按"相邻间隔是否正常"筛出可信样本。
  std::vector<std::size_t> kept;
  kept.reserve(window.size());
  for (std::size_t i = 1; i + 1 < window.size(); ++i) {
    const double left = window[i].t - window[i - 1].t;
    const double right = window[i + 1].t - window[i].t;
    if (!(median_dt > 0.0) || left < 0.5 * median_dt || left > 1.5 * median_dt ||
      right < 0.5 * median_dt || right > 1.5 * median_dt)
    {
      continue;
    }
    kept.push_back(i);
  }
  const unsigned int rejected = static_cast<unsigned int>(window.size()) -
    static_cast<unsigned int>(kept.size());
  if (rejected > 0) {
    RCLCPP_INFO(
      get_logger(), "剔除 %u 个间隔异常的样本（正常步长中位数 %.5f s）", rejected, median_dt);
  }

  // 第二遍：中心差分的邻居只在**筛过之后**的样本里取。
  // 否则一个被剔除的坏样本（时间戳错位 → q̇ 毛刺）仍会被相邻点当作邻居，
  // 直接把 q̈ 放大成尖刺 —— 实测残差 MAX 的另一半原因就在这里。
  std::vector<Record> used;
  std::vector<double> qddot;
  used.reserve(kept.size());
  qddot.reserve(kept.size() * n_);
  for (std::size_t k = 1; k + 1 < kept.size(); ++k) {
    const Record & previous = window[kept[k - 1]];
    const Record & current = window[kept[k]];
    const Record & next = window[kept[k + 1]];
    const double dt = next.t - previous.t;
    if (!(dt > 0.0)) {
      continue;
    }
    for (unsigned int j = 0; j < n_; ++j) {
      qddot.push_back((next.qdot(j) - previous.qdot(j)) / dt);
    }
    used.push_back(current);
  }

  std::string message;
  if (!solveAndReport(used, qddot, message)) {
    RCLCPP_ERROR(get_logger(), "辨识失败：%s", message.c_str());
    return;
  }

  const std::string samples_path = std::filesystem::absolute(samples_csv_).string();
  const std::string params_path = std::filesystem::absolute(parameters_csv_).string();
  if (writeSamplesCsv(used, qddot, last_prediction_, samples_path)) {
    RCLCPP_INFO(get_logger(), "样本已写入：%s", samples_path.c_str());
  }
  RCLCPP_INFO(get_logger(), "参数已写入：%s", params_path.c_str());
}

bool IdentifyNode::solveAndReport(
  const std::vector<Record> & window, const std::vector<double> & qddot, std::string & message)
{
  const std::size_t count = window.size();
  const unsigned int inertia_columns = kParamsPerLink * builder_->segments();
  const unsigned int friction_columns = use_friction_model_ ? 2 * n_ : 0;
  const unsigned int total_columns = inertia_columns + friction_columns;

  // 组装采样点与实测力矩。
  std::vector<RegressorSample> samples(count);
  Eigen::VectorXd torque(static_cast<Eigen::Index>(count * n_));
  for (std::size_t i = 0; i < count; ++i) {
    samples[i].q = KDL::JntArray(n_);
    samples[i].qdot = KDL::JntArray(n_);
    samples[i].qddot = KDL::JntArray(n_);
    for (unsigned int j = 0; j < n_; ++j) {
      samples[i].q(j) = window[i].q(j);
      samples[i].qdot(j) = window[i].qdot(j);
      samples[i].qddot(j) = qddot[i * n_ + j];
      torque(static_cast<Eigen::Index>(i * n_ + j)) = window[i].tau(j);
    }
  }

  const RegressorResult regressor = builder_->build(samples);
  if (!regressor.success) {
    message = "构建回归矩阵失败：" + regressor.message;
    return false;
  }

  // 摩擦列：第 j 个关节两列 —— 粘性 diag(q̇)、库伦 diag(sign(q̇))。
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(regressor.rows(), total_columns);
  augmented.leftCols(inertia_columns) = regressor.regressor;
  if (use_friction_model_) {
    for (std::size_t i = 0; i < count; ++i) {
      for (unsigned int j = 0; j < n_; ++j) {
        const double v = samples[i].qdot(j);
        const Eigen::Index row = static_cast<Eigen::Index>(i * n_ + j);
        augmented(row, inertia_columns + j) = v;
        augmented(row, inertia_columns + n_ + j) = (v > 0.0) ? 1.0 : ((v < 0.0) ? -1.0 : 0.0);
      }
    }
  }

  LeastSquaresOptions options;
  options.rank_tolerance = rank_tolerance_;
  const IdentificationResult solution = solveLeastSquares(augmented, torque, options);
  if (!solution.success) {
    message = "最小二乘求解失败：" + solution.message;
    return false;
  }
  last_prediction_ = solution.prediction;  // 交给 writeSamplesCsv 写进 CSV，供画图对比

  RCLCPP_INFO(
    get_logger(),
    "\n===== 辨识结果 =====\n"
    "  样本数        : %zu（%u 关节）\n"
    "  参数列        : 惯性 %u + 摩擦 %u = %u\n"
    "  可辨识基参数  : %u\n"
    "  力矩残差 RMS  : %.3e N·m\n"
    "  力矩残差 MAX  : %.3e N·m\n"
    "  基参数条件数  : %.3e",
    count, n_, inertia_columns, friction_columns, total_columns, solution.rank,
    solution.residual_rms, solution.residual_max, solution.scaled_condition_number);

  // 最大残差出现在哪个采样点 / 关节（定位异常样本，如接触或限位）。
  {
    Eigen::Index max_index = 0;
    (torque - solution.prediction).cwiseAbs().maxCoeff(&max_index);
    const unsigned int sample_index = static_cast<unsigned int>(max_index) / n_;
    const unsigned int joint_index = static_cast<unsigned int>(max_index) % n_;
    RCLCPP_INFO(
      get_logger(), "  最大残差：t=%.3f s, %s，实测 %+.3f / 预测 %+.3f N·m",
      window[sample_index].t, joint_names_[joint_index].c_str(), torque(max_index),
      solution.prediction(max_index));
  }

  // 摩擦系数（若启用）：与 MJCF 的 damping 对照。
  if (use_friction_model_) {
    RCLCPP_INFO(get_logger(), "  摩擦辨识（粘性 / 库伦）：");
    for (unsigned int j = 0; j < n_; ++j) {
      RCLCPP_INFO(
        get_logger(), "    %-7s 粘性 = %+.4f   库伦 = %+.4f", joint_names_[j].c_str(),
        solution.parameters(inertia_columns + j),
        solution.parameters(inertia_columns + n_ + j));
    }
  }

  // 基参数（惯性部分）逐项打印。
  RCLCPP_INFO(get_logger(), "  基参数（惯性部分）：");
  for (unsigned int index : solution.base_parameter_indices) {
    if (index < inertia_columns) {
      RCLCPP_INFO(
        get_logger(), "    %-16s = %+.6e", parameterLabel(chain_, index).c_str(),
        solution.parameters(index));
    }
  }

  // 参数 CSV。
  {
    const std::string path = std::filesystem::absolute(parameters_csv_).string();
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    std::ofstream csv(path);
    if (csv) {
      csv << "index,label,value\n";
      for (unsigned int index : solution.base_parameter_indices) {
        std::string label;
        if (index < inertia_columns) {
          label = parameterLabel(chain_, index);
        } else if (index < inertia_columns + n_) {
          label = joint_names_[index - inertia_columns] + ".viscous_friction";
        } else {
          label = joint_names_[index - inertia_columns - n_] + ".coulomb_friction";
        }
        csv << index << "," << label << "," << solution.parameters(index) << "\n";
      }
    }
  }

  message.clear();
  return true;
}

bool IdentifyNode::writeSamplesCsv(
  const std::vector<Record> & window, const std::vector<double> & qddot,
  const Eigen::VectorXd & prediction, const std::string & path) const
{
  // 输出目录可能还不存在（比如空目录被清理过），先建出来。
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }
  const bool has_prediction =
    prediction.size() == static_cast<Eigen::Index>(window.size() * n_);
  csv << "t";
  for (unsigned int j = 0; j < n_; ++j) {
    csv << ",q" << j;
  }
  for (unsigned int j = 0; j < n_; ++j) {
    csv << ",qdot" << j;
  }
  for (unsigned int j = 0; j < n_; ++j) {
    csv << ",qddot" << j;
  }
  for (unsigned int j = 0; j < n_; ++j) {
    csv << ",tau" << j;
  }
  if (has_prediction) {
    for (unsigned int j = 0; j < n_; ++j) {
      csv << ",tau_pred" << j;
    }
  }
  csv << "\n";
  for (std::size_t i = 0; i < window.size(); ++i) {
    csv << window[i].t;
    for (unsigned int j = 0; j < n_; ++j) {
      csv << "," << window[i].q(j);
    }
    for (unsigned int j = 0; j < n_; ++j) {
      csv << "," << window[i].qdot(j);
    }
    for (unsigned int j = 0; j < n_; ++j) {
      csv << "," << qddot[i * n_ + j];
    }
    for (unsigned int j = 0; j < n_; ++j) {
      csv << "," << window[i].tau(j);
    }
    if (has_prediction) {
      for (unsigned int j = 0; j < n_; ++j) {
        csv << "," << prediction(static_cast<Eigen::Index>(i * n_ + j));
      }
    }
    csv << "\n";
  }
  return true;
}

}  // namespace kdl_identification

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kdl_identification::IdentifyNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("kdl_identify"), "节点启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
