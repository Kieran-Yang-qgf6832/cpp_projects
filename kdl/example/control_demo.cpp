// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务层（control）的完整示例。
//
// 链路：URDF → RobotContext → 任务信号(TaskRequest) → TaskRouter 解算 → 关节空间轨迹
//
// 分节对应 src/control/PLAN.md 的验收标准：
//   1) 装配上下文
//   2) 分派的健壮性（未实现的任务、尺寸错误、目标越界都必须是"失败 + 原因"）
//   3) 任务1：关节空间运动（自动定时是怎么算出来的、固定时长过快怎么报错）
//   4) 任务2：笛卡尔空间运动（单段/多路点、kStop/kPassThrough、FK 闭环自检）
//   5) 动力学：力矩前馈、与重力项对照、力矩上限超限的负向用例
//   6) 导出 CSV

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_cartesian.hpp"
#include "router/kdl_control_print.hpp"
#include "kdl_dynparam.hpp"
#include "kdl_fk.hpp"
#include "kdl_interpolation_print.hpp"
#include "tasks/kdl_joint_space_task.hpp"
#include "kdl_quintic.hpp"
#include "router/kdl_task_router.hpp"

// 由 CMake 在编译期传入（见 CMakeLists.txt 的 KDL_TOOLS_MODEL_DIR）。
#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

namespace
{

constexpr unsigned int kCsvSamples = 201;

/// 由初始化列表构造关节向量，省掉一堆 resize + 下标赋值。
KDL::JntArray jnt(std::initializer_list<double> values)
{
  KDL::JntArray q(static_cast<unsigned int>(values.size()));
  unsigned int i = 0;
  for (double v : values) {
    q(i++) = v;
  }
  return q;
}

/// 给某个关节向量统一赋值（用于设置限位）。
KDL::JntArray filled(unsigned int n, double value)
{
  KDL::JntArray q(n);
  for (unsigned int i = 0; i < n; ++i) {
    q(i) = value;
  }
  return q;
}

void printVector(const char * label, const KDL::JntArray & q)
{
  std::cout << "  " << label;
  kdl_interpolation::printJointVector(q);  // 自带换行
}

/// 把关节轨迹等间隔采样后写成 CSV（列名沿用 quintic_demo 的"前缀 + 关节号"约定）。
bool writeJointCsv(
  const kdl_interpolation::QuinticTrajectory & trajectory, const std::string & path)
{
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }

  const unsigned int joints = trajectory.joints();
  csv << "t";
  for (unsigned int j = 0; j < joints; ++j) {
    csv << ",q" << j;
  }
  for (unsigned int j = 0; j < joints; ++j) {
    csv << ",qdot" << j;
  }
  for (unsigned int j = 0; j < joints; ++j) {
    csv << ",qddot" << j;
  }
  csv << "\n";

  const double total = trajectory.duration();
  for (unsigned int k = 0; k < kCsvSamples; ++k) {
    const double t = total * static_cast<double>(k) / static_cast<double>(kCsvSamples - 1);
    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qddot;
    if (!trajectory.sample(t, q, qdot, qddot)) {
      return false;
    }
    csv << t;
    for (unsigned int j = 0; j < joints; ++j) {
      csv << "," << q(j);
    }
    for (unsigned int j = 0; j < joints; ++j) {
      csv << "," << qdot(j);
    }
    for (unsigned int j = 0; j < joints; ++j) {
      csv << "," << qddot(j);
    }
    csv << "\n";
  }
  return true;
}

/**
 * @brief 导出"笛卡尔参考位姿 vs 关节轨迹 FK"的对照表。
 * @note 这里的参考轨迹是示例自己按同样的路点与时长重新构建的（用的都是公开接口），
 *       目的只是把对照曲线导出来看；**任务内部的自检结论以
 *       ControlResult::position_error / orientation_error 为准**，两者用的是同一套判据。
 */
bool writeCartesianCsv(
  const kdl_interpolation::CartesianTrajectory & reference,
  const kdl_interpolation::QuinticTrajectory & joint_trajectory, const KDL::Chain & chain,
  const std::string & path)
{
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }

  csv << "t,ref_x,ref_y,ref_z,fk_x,fk_y,fk_z,pos_err,rot_err\n";

  const double total = reference.duration();
  for (unsigned int k = 0; k < kCsvSamples; ++k) {
    const double t = total * static_cast<double>(k) / static_cast<double>(kCsvSamples - 1);

    kdl_interpolation::CartesianState state;
    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qddot;
    KDL::Frame fk;
    if (!reference.sample(t, state) || !joint_trajectory.sample(t, q, qdot, qddot) ||
        !kdl_kinematics::forwardKinematics(chain, q, fk))
    {
      return false;
    }

    KDL::Vector axis;
    const double rot_err = (state.pose.M.Inverse() * fk.M).GetRotAngle(axis, 1e-8);
    csv << t << "," << state.pose.p.x() << "," << state.pose.p.y() << "," << state.pose.p.z() << ","
        << fk.p.x() << "," << fk.p.y() << "," << fk.p.z() << "," << (fk.p - state.pose.p).Norm()
        << "," << rot_err << "\n";
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("control_demo");
  std::cout << std::setprecision(6);

  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";

  // =========================================================================
  // 1) 装配任务上下文
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "1) 装配 RobotContext（URDF → 链 + 行程 + 力矩上限）\n"
            << "=============================================================\n";

  kdl_control::TaskRouter router;
  kdl_control::RobotContext ctx;
  std::string message;
  if (!kdl_control::TaskRouter::loadContextFromUrdf(urdf_file, ctx, message)) {
    std::cerr << "  装配失败: " << message << "\n";
    rclcpp::shutdown();
    return 1;
  }

  // URDF 只提供行程与力矩上限。速度/加速度/jerk 与末端限位是"使用场景"的信息，
  // 必须由调用者显式给出 —— 自动定时全靠它们，不给就只能显式指定时长。
  const unsigned int n = ctx.chain.getNrOfJoints();
  ctx.joint_limits.max_velocity = filled(n, 1.0);  // rad/s

  // 加速度上限**不能所有关节一刀切**：靠近末端的关节力臂只有 0.1 m 量级，末端要做出
  // 2 m/s² 的加速度，该关节就得转 20 rad/s² 左右（q̈ ≈ a / r）。这也是笛卡尔限位
  // 满足之后、关节侧仍然可能判"不可行"的原因 —— 两个空间的量纲不是一回事。
  // 这里前三个关节取 30，末端关节取 80（力臂最短，需要更高的关节角加速度）。
  ctx.joint_limits.max_acceleration = jnt({ 30.0, 30.0, 30.0, 80.0 });  // rad/s²
  ctx.joint_limits.max_jerk = filled(n, 100.0);  // rad/s³（单段示例的端点 jerk ≈ 25，可过）
  ctx.cartesian_limits.max_linear_velocity = 0.5;           // m/s
  ctx.cartesian_limits.max_linear_acceleration = 1.0;       // m/s²
  ctx.cartesian_limits.max_angular_velocity = 1.0;          // rad/s
  ctx.cartesian_limits.max_angular_acceleration = 2.0;      // rad/s²
  router.setContext(ctx);
  kdl_control::printRobotContext(ctx);

  std::cout << "  动力学可用性自检: ";
  if (kdl_control::TaskRouter::checkDynamicsAvailable(ctx, message)) {
    std::cout << "通过（链上有惯量参数，可以算力矩前馈）\n";
  } else {
    std::cout << "不通过 —— " << message << "\n";
  }

  // 起始状态：**刻意不取全零位**。零位是"直臂"构型，最接近奇异
  // （实测雅可比最小奇异值 0.0185，正常构型 0.18~0.21）。在那里逐点逆解不唯一，
  // 解会在零空间方向乱窜，重建出来的速度/加速度没有物理意义。第 4c 节专门演一遍。
  const KDL::JntArray q_start = jnt({ 0.0, -0.6, 0.8, 0.0 });
  const KDL::JntArray q_zero = filled(n, 0.0);  // 只用于奇异位形的负向用例与重力项对照
  const KDL::JntArray qdot_zero = filled(n, 0.0);
  // 关节空间任务的目标构型。
  const KDL::JntArray q_goal = jnt({ 0.5, -0.4, 0.6, 0.3 });

  // =========================================================================
  // 2) 分派的健壮性：三类"应当失败"的输入
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "2) 分派健壮性（都应当是「失败 + 原因」，而不是崩溃）\n"
            << "=============================================================\n";

  {  // (a) 未实现的任务类型
    kdl_control::TaskRequest req;
    req.type = kdl_control::TaskType::kUnknown;
    const auto result = router.dispatch(req, q_zero, qdot_zero);
    std::cout << "  (a) 未实现的任务类型:\n";
    kdl_control::printControlResult(result);
  }

  {  // (b) 状态向量长度不对
    kdl_control::TaskRequest req;
    req.type = kdl_control::TaskType::kJointSpace;
    const KDL::JntArray q_wrong = filled(n - 1, 0.0);
    const auto result = router.dispatch(req, q_wrong, qdot_zero);
    std::cout << "  (b) 当前关节角长度少 1:\n";
    kdl_control::printControlResult(result);
  }

  {  // (c) 目标超出关节行程
    kdl_control::TaskRequest req;
    req.type = kdl_control::TaskType::kJointSpace;
    req.goal_joint = q_goal;
    req.goal_joint(2) = ctx.q_max(2) + 1.0;  // 第 3 个关节故意越界
    const auto result = router.dispatch(req, q_zero, qdot_zero);
    std::cout << "  (c) 第 3 个关节目标越界:\n";
    kdl_control::printControlResult(result);
  }

  // =========================================================================
  // 3) 任务1：关节空间运动
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "3) 任务1：关节空间运动（duration = 0 → 自动定时）\n"
            << "=============================================================\n";

  kdl_control::TaskRequest joint_req;
  joint_req.type = kdl_control::TaskType::kJointSpace;
  joint_req.name = "homing_to_goal";
  joint_req.goal_joint = q_goal;
  joint_req.compute_torque_feedforward = true;  // 第 5 节会看结果

  // 先手工复算一遍"自动定时"的来路，让结论肉眼可查：
  //   T ≥ 1.875·|Δq_i| / v_max_i，逐关节算完取最大者。
  double auto_duration = 0.0;
  if (!kdl_control::JointSpaceTask::estimateDuration(
        ctx, q_start, q_goal, auto_duration, message))
  {
    std::cerr << "  自动定时失败: " << message << "\n";
  } else {
    std::cout << "  自动定时的来路（1.875 = 15/8，来自 kdl_quintic.hpp 文件头）:\n"
              << "    关节    |Δq| [rad]    v_max [rad/s]    1.875·|Δq|/v_max [s]\n";
    double longest = 0.0;
    for (unsigned int i = 0; i < n; ++i) {
      const double delta = std::abs(q_goal(i) - q_start(i));
      const double needed = 15.0 / 8.0 * delta / ctx.joint_limits.max_velocity(i);
      longest = std::max(longest, needed);
      std::cout << "      " << (i + 1) << std::setw(14) << delta << std::setw(18)
                << ctx.joint_limits.max_velocity(i) << std::setw(26) << needed << "\n";
    }
    std::cout << "    取最大值 → T = " << longest << " s（估计值 " << auto_duration << " s）\n";
  }

  const auto joint_result = router.dispatch(joint_req, q_start, qdot_zero);
  kdl_control::printControlResult(joint_result);

  // 负向用例：固定一个过短的时长，插值模块应当报"某关节超速"而不是静默成功。
  {
    std::cout << "\n  负向用例：把 duration 固定成 0.05 s（位移不变）\n";
    kdl_control::TaskRequest fast_req = joint_req;
    fast_req.duration = 0.05;
    fast_req.compute_torque_feedforward = false;
    const auto fast_result = router.dispatch(fast_req, q_start, qdot_zero);
    kdl_control::printControlResult(fast_result);
  }

  // =========================================================================
  // 4) 任务2：笛卡尔空间运动
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "4) 任务2：笛卡尔空间运动\n"
            << "=============================================================\n";

  // 目标位姿直接用 FK(q_goal) 造出来，保证"确实可达"。
  KDL::Frame goal_pose;
  if (!kdl_kinematics::forwardKinematics(ctx.chain, q_goal, goal_pose)) {
    std::cerr << "  FK 失败，无法构造笛卡尔目标\n";
    rclcpp::shutdown();
    return 1;
  }
  KDL::Frame start_pose;
  if (!kdl_kinematics::forwardKinematics(ctx.chain, q_start, start_pose)) {
    std::cerr << "  FK 失败，无法构造笛卡尔起点\n";
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "  起点（FK(q_start)）: p = (" << start_pose.p.x() << ", " << start_pose.p.y()
            << ", " << start_pose.p.z() << ")\n";
  std::cout << "  目标（FK(q_goal)）: p = (" << goal_pose.p.x() << ", " << goal_pose.p.y() << ", "
            << goal_pose.p.z() << ")，位移 " << (goal_pose.p - start_pose.p).Norm() << " m\n";

  // ---- 4a. 单段：位置走直线 + 姿态绕固定轴转最短弧；时长自动估算 ----
  std::cout << "\n  --- 4a) 单段（duration = 0 → 自动定时；位置走直线）\n";
  kdl_control::TaskRequest cart_req;
  cart_req.type = kdl_control::TaskType::kCartesianSpace;
  cart_req.name = "cart_single";
  cart_req.goal_pose = goal_pose;
  cart_req.has_start_pose = false;  // 让任务自己用 FK(q_now) 求起点
  // sample_dt 控制"多密地采样参考路径并逆解"，rebuild_dt 控制"用多稀的路点串成
  // C⁴ 轨迹"。两者分工的原因写在 TaskRequest::rebuild_dt 的注释里（10 ms 的段长
  // 承载不了 C⁴ 条件，解出来的加速度会被逆解噪声主导）。

  const auto cart_single = router.dispatch(cart_req, q_start, qdot_zero);
  kdl_control::printControlResult(cart_single);

  // 把这条参考轨迹按同样的时长重建一份，仅用于导出对照 CSV（见 writeCartesianCsv 注释）。
  std::vector<KDL::Frame> single_path{ start_pose, goal_pose };
  const std::vector<double> single_durations{ cart_single.duration };
  const auto single_reference = kdl_interpolation::buildCartesianTrajectory(
    single_path, single_durations, cart_req.behavior, ctx.cartesian_limits);

  // ---- 4b. 多路点：故意让末端"不沿直线运动" ----
  std::cout << "\n  --- 4b) 三路点：绕行路点用「关节空间中点」构造，看两种路点行为\n";
  // 中间路点怎么选：取关节空间的中点再正解回位姿。好处是①一定可达；②一般不会把臂
  // 带进奇异区（随手写个"位置偏一点"的中间点很容易踩到，见 4c）；③末端路径自然
  // 就不沿直线。手写位置偏移是对称性没保证的做法，教学上不如这个稳。
  KDL::JntArray q_mid(n);
  for (unsigned int i = 0; i < n; ++i) {
    q_mid(i) = 0.5 * (q_start(i) + q_goal(i));
  }
  KDL::Frame mid_pose;
  if (!kdl_kinematics::forwardKinematics(ctx.chain, q_mid, mid_pose)) {
    std::cerr << "  FK 失败，无法构造中间路点\n";
    rclcpp::shutdown();
    return 1;
  }

  std::vector<KDL::Frame> detour_path{ start_pose, mid_pose, goal_pose };
  {
    // 中间点到"起止直线"的距离（点到直线距离，和后面量参考路径用的是同一个口径）
    const KDL::Vector line = goal_pose.p - start_pose.p;
    const double line_len2 = KDL::dot(line, line);
    const double s_mid =
      (line_len2 > 0.0) ? KDL::dot(mid_pose.p - start_pose.p, line) / line_len2 : 0.0;
    std::cout << "    中间点 p = (" << mid_pose.p.x() << ", " << mid_pose.p.y() << ", "
              << mid_pose.p.z() << ")，到起止直线的距离 "
              << (mid_pose.p - (start_pose.p + s_mid * line)).Norm() << " m\n";
  }

  for (const auto behavior :
       { kdl_interpolation::WaypointBehavior::kStop, kdl_interpolation::WaypointBehavior::kPassThrough })
  {
    kdl_control::TaskRequest detour_req;
    detour_req.type = kdl_control::TaskType::kCartesianSpace;
    detour_req.name = (behavior == kdl_interpolation::WaypointBehavior::kStop) ? "cart_detour_stop"
                                                                             : "cart_detour_pass";
    detour_req.cartesian_waypoints = { mid_pose, goal_pose };  // 起点由任务插入
    detour_req.durations = { 2.0, 2.5 };                       // 两段给不同时长
    detour_req.behavior = behavior;

    std::cout << "\n    [" << (behavior == kdl_interpolation::WaypointBehavior::kStop ? "kStop" :
                                                                                      "kPassThrough")
              << "] 段时长 2.0 s + 2.5 s\n";
    const auto detour_result = router.dispatch(detour_req, q_start, qdot_zero);
    kdl_control::printControlResult(detour_result);

    if (behavior == kdl_interpolation::WaypointBehavior::kPassThrough) {
      // 顺便量一下"不沿直线"这件事：参考路径相对起止直线的最大偏离。
      const auto reference = kdl_interpolation::buildCartesianTrajectory(
        detour_path, detour_req.durations, behavior, ctx.cartesian_limits);
      double max_deviation = 0.0;
      if (reference.success) {
        for (unsigned int k = 0; k <= 50; ++k) {
          kdl_interpolation::CartesianState state;
          const double t = reference.trajectory.duration() * static_cast<double>(k) / 50.0;
          if (!reference.trajectory.sample(t, state)) {
            continue;
          }
          // 点到直线 p0 + s·(p1-p0) 的距离
          // 注意：KDL 的向量 operator* 是叉乘，点乘要用 KDL::dot()。
          const KDL::Vector dir = goal_pose.p - start_pose.p;
          const double len2 = KDL::dot(dir, dir);
          const KDL::Vector rel = state.pose.p - start_pose.p;
          const double s = (len2 > 0.0) ? KDL::dot(rel, dir) / len2 : 0.0;
          const KDL::Vector closest = start_pose.p + s * dir;
          max_deviation = std::max(max_deviation, (state.pose.p - closest).Norm());
        }
        std::cout << "    参考路径相对起止直线的最大偏离 = " << max_deviation
                  << " m（> 0 说明「不沿直线运动」确实发生了）\n";
      }
    }
  }

  // ---- 4c. 负向用例：起点取在奇异位形（零位 = 直臂）----
  std::cout << "\n  --- 4c) 负向用例：起点取在奇异位形（零位 = 直臂）\n";
  {
    kdl_control::TaskRequest singular_req;
    singular_req.type = kdl_control::TaskType::kCartesianSpace;
    singular_req.name = "cart_from_singular";
    singular_req.goal_pose = goal_pose;  // 目标不变，只把起点换到奇异位形

    const auto singular_result = router.dispatch(singular_req, q_zero, qdot_zero);
    kdl_control::printControlResult(singular_result);
    std::cout << "    说明：零位是直臂构型（最小奇异值 ≈ 0.0185，工作构型 0.18~0.21），\n"
              << "          比工作构型更接近奇异，逐点逆解在那里更不稳定。\n"
              << "          解算如实拒绝，而不是交出一条看上去能跑、实际乱抖的轨迹。\n";
  }

  // =========================================================================
  // 5) 动力学：力矩前馈、与重力项对照、力矩上限超限
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "5) 动力学：力矩前馈与力矩上限\n"
            << "=============================================================\n";

  kdl_control::printTorqueFeedforward(joint_result);

  // 起点静止时 τ 只剩重力项，可以拿 kdl_dynamics 的重力项对照。
  const auto gravity = kdl_dynamics::gravityTorque(ctx.chain, q_start);
  if (gravity.error_code >= 0) {
    printVector("自重下的重力项 G(q_start) [N·m]: ", gravity.torque);
    std::cout << "  说明：t = 0 时轨迹速度、加速度都为 0，所以前馈 τ 与重力项同量级；\n"
              << "        运动中间段才会多出 M·q̈ 与 C·q̇ 两项。\n";
  }

  {
    // 负向用例：人为把力矩上限压到 1 N·m，应当判为"力矩超限"并指出关节与时刻。
    std::cout << "\n  负向用例：把力矩上限人工压到 1 N·m，其余不变\n";
    kdl_control::RobotContext tight_ctx = ctx;
    for (unsigned int i = 0; i < n; ++i) {
      tight_ctx.max_torque(i) = 1.0;
    }
    router.setContext(tight_ctx);

    kdl_control::TaskRequest tight_req = joint_req;
    tight_req.check_torque_limit = true;
    const auto tight_result = router.dispatch(tight_req, q_start, qdot_zero);
    kdl_control::printControlResult(tight_result);

    router.setContext(ctx);  // 复原
  }

  // =========================================================================
  // 6) 导出 CSV
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << "6) 导出 CSV\n"
            << "=============================================================\n";

  if (joint_result.success) {
    const std::string path =
      std::filesystem::absolute("control_joint_trajectory.csv").string();
    if (writeJointCsv(joint_result.joint_trajectory, path)) {
      std::cout << "  已写入: " << path << "\n"
                << "  列含义: t, q0..q" << (n - 1) << ", qdot0..qdot" << (n - 1)
                << ", qddot0..qddot" << (n - 1) << "（" << kCsvSamples
                << " 行等间隔采样）\n";
    } else {
      std::cout << "  写入失败: " << path << "\n";
    }
  } else {
    std::cout << "  任务1 未成功，跳过关节轨迹导出\n";
  }

  if (cart_single.success && single_reference.success) {
    const std::string path =
      std::filesystem::absolute("control_cartesian_check.csv").string();
    if (writeCartesianCsv(
          single_reference.trajectory, cart_single.joint_trajectory, ctx.chain, path))
    {
      std::cout << "  已写入: " << path << "\n"
                << "  列含义: t, ref_x/y/z（笛卡尔参考位置）, fk_x/y/z（关节轨迹 FK 位置）, "
                   "pos_err, rot_err\n";
    } else {
      std::cout << "  写入失败: " << path << "\n";
    }
  } else {
    std::cout << "  笛卡尔任务未成功，跳过对照导出\n";
  }

  std::cout << "\n示例结束。\n";
  rclcpp::shutdown();
  return 0;
}
