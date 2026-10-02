// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务2（笛卡尔空间运动）的实现。

#include "tasks/kdl_cartesian_space_task.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "kdl_fk.hpp"
#include "kdl_ik.hpp"
#include "kdl_velocity.hpp"

namespace kdl_control
{
namespace
{

/// 单段峰值速度系数：|v|max = 15·Δ/(8T)、|ω|max = 15·θ/(8T)（kdl_cartesian.hpp 文件头）。
constexpr double kPeakVelocityFactor = 15.0 / 8.0;

/// 单段峰值加速度系数：|a|max = 10·Δ/(√3·T²)、|α|max = 10·θ/(√3·T²)。
constexpr double kPeakAccelerationFactor = 10.0 / std::sqrt(3.0);

/// IK 收敛精度（任务空间误差）与最大迭代次数。
constexpr double kIkEps = 1e-5;
constexpr unsigned int kIkMaxIter = 500;

/// 段时长兜底值 [s]：起点与终点位姿几乎相同时用，避免生成 T = 0 的非法段。
constexpr double kMinSegmentDuration = 0.1;

/// 自动定时的安全余量，理由见 kdl_joint_space_task.cpp 里的同名常量。
constexpr double kDurationSafetyFactor = 1.02;

/// 采样步长兜底值 [s]。
constexpr double kDefaultSampleDt = 0.01;

/**
 * @brief "近奇异位形"判据：雅可比最小奇异值的下限。
 *
 * @note 阈值是本机器人量出来的，量级很清楚（见示例打印）：
 *         零位（直臂，典型奇异位形）= 0.0011
 *         弯曲构型                    = 0.16
 *         目标构型                    = 0.12
 *       所以取 0.01 —— 比正常构型低一个多数量级，只有真的贴到奇异面上才会触发。
 * @note 为什么必须在这里挡一下：接近奇异时逆解在零空间方向几乎不改变末端位姿，
 *       于是逐点逆解可以"合法地"在零空间里跳来跳去（实测 10 ms 内跳 0.017 rad，
 *       而末端只动了 9e-5 m）。插值重建会把这些跳变放大成巨大的速度/加速度/jerk，
 *       最后表现为"轨迹不可行"这种看不出真因的报错。在这里如实说清楚，
 *       比让人去猜"为什么加速度虚高 40 rad/s²"要省事得多。
 */
constexpr double kSingularityThreshold = 0.01;

/// 把数值格式化成可读字符串（只用于拼错误信息）。
std::string num(double value, int precision = 6)
{
  std::ostringstream os;
  os << std::setprecision(precision) << value;
  return os.str();
}

/**
 * @brief 两个旋转之间的夹角（最短弧），单位 rad。
 *
 * @note 用 KDL 自带的 GetRotAngle：它内部会处理"夹角接近 0"的退化情形，
 *       不需要自己写 atan2 去防除零。
 */
double rotationAngle(const KDL::Rotation & from, const KDL::Rotation & to)
{
  KDL::Vector axis;
  return (from.Inverse() * to).GetRotAngle(axis, 1e-8);
}

/**
 * @brief 一次"离散 + IK + 重建 + 自检"的结果（内部使用）。
 */
struct Attempt
{
  bool success = false;
  std::string message;
  kdl_interpolation::QuinticTrajectory trajectory{};
  double position_error = 0.0;     ///< FK 自检位置残差峰值 [m]
  double orientation_error = 0.0;  ///< FK 自检姿态残差峰值 [rad]
  unsigned int ik_failures = 0;    ///< IK 未收敛点数
  unsigned int samples = 0;        ///< 实际采样点数（细网格）
  unsigned int rebuild_knots = 0;  ///< 重建轨迹使用的路点数（稀疏网格）
  double sample_dt = 0.0;          ///< 实际使用的采样步长 [s]
  double min_singular_value = 1e9; ///< 整条路径上雅可比最小奇异值的最小值（离奇异面多远）
};

/**
 * @brief 把笛卡尔轨迹变成关节轨迹，并做闭环自检（详见头文件"难点"一节）。
 * @param ctx       [in] 机器人上下文。
 * @param req       [in] 任务信号（取阈值等）。
 * @param cartesian [in] 已构建好的笛卡尔参考轨迹。
 * @param q_now     [in] 当前关节角（作为第一个 IK 点的初值，保证与实际状态一致）。
 * @param sample_dt [in] 期望的采样步长 [s]。
 * @return Attempt；失败时 message 说明原因。
 */
Attempt runAttempt(
  const RobotContext & ctx, const TaskRequest & req,
  const kdl_interpolation::CartesianTrajectory & cartesian, const KDL::JntArray & q_now,
  double sample_dt)
{
  Attempt attempt;

  const unsigned int n = ctx.chain.getNrOfJoints();
  const double total = cartesian.duration();
  if (total <= 0.0) {
    attempt.message = "笛卡尔轨迹总时长为 0，无法离散";
    return attempt;
  }

  // ---- 1. 均匀采样：间隔数按 sample_dt 算，再受上限约束，保证末点恰好落在 total ----
  const double dt_requested = sample_dt > 0.0 ? sample_dt : kDefaultSampleDt;
  unsigned int intervals = static_cast<unsigned int>(std::ceil(total / dt_requested));
  if (intervals < 1) {
    intervals = 1;
  }
  if (intervals + 1 > CartesianSpaceTask::kMaxSamples) {
    intervals = CartesianSpaceTask::kMaxSamples - 1;
  }
  const double dt = total / static_cast<double>(intervals);

  std::vector<double> times(intervals + 1, 0.0);
  std::vector<KDL::Frame> references(intervals + 1);
  for (unsigned int k = 0; k <= intervals; ++k) {
    times[k] = (k == intervals) ? total : dt * static_cast<double>(k);
    kdl_interpolation::CartesianState state;
    if (!cartesian.sample(times[k], state)) {
      attempt.message = "笛卡尔轨迹采样失败（t = " + num(times[k]) + " s）";
      return attempt;
    }
    references[k] = state.pose;
  }

  // ---- 2. 逐点 IK：热启动（初值取上一点的解）----
  // 热启动有两个好处：一是收敛快，二是解在采样点之间连续，不会在不同构型支之间乱跳。
  // 第一个点的初值必须是**当前关节角**（而不是零位），否则第一步就可能跳到别的解上。
  std::vector<KDL::JntArray> joints(intervals + 1);
  KDL::JntArray q_init = q_now;
  for (unsigned int k = 0; k <= intervals; ++k) {
    const kdl_kinematics::IkResult ik =
      kdl_kinematics::solveIkLma(ctx.chain, q_init, references[k], kIkEps, kIkMaxIter);
    joints[k] = ik.q;

    if (!ik.success()) {
      ++attempt.ik_failures;
      attempt.message = "第 " + std::to_string(k) + " 个采样点（t = " + num(times[k]) +
                        " s）IK 未收敛：" + ik.message + "；目标位姿可能超出工作空间";
      return attempt;
    }

    if (k > 0) {
      double jump = 0.0;
      unsigned int jump_joint = 0;
      for (unsigned int i = 0; i < n; ++i) {
        const double delta = std::abs(joints[k](i) - joints[k - 1](i));
        if (delta > jump) {
          jump = delta;
          jump_joint = i;
        }
      }
      if (jump > req.joint_jump_threshold) {
        attempt.message = "第 " + std::to_string(k) + " 个采样点出现关节跳变：关节 " +
                          std::to_string(jump_joint + 1) + " 一步动了 " + num(jump) +
                          " rad（阈值 " + num(req.joint_jump_threshold) +
                          "），疑似翻关节或绕过奇异；请增加中间路点，或改用别的逆解支";
        return attempt;
      }
    }

    // 近奇异检查：位形太靠近奇异面时，后面的插值重建没有意义（见 kSingularityThreshold）
    const double sigma_min = kdl_kinematics::minSingularValue(ctx.chain, joints[k]);
    attempt.min_singular_value =
      (k == 0) ? sigma_min : std::min(attempt.min_singular_value, sigma_min);
    if (sigma_min < kSingularityThreshold) {
      attempt.message = "第 " + std::to_string(k) + " 个采样点（t = " + num(times[k]) +
                        " s）的位形接近奇异：雅可比最小奇异值 " + num(sigma_min) + " < " +
                        num(kSingularityThreshold) +
                        "，此处关节解在零空间方向几乎不影响末端位姿，逐点逆解会不连续、"
                        "重建出的速度/加速度不可信；请避开奇异位形（例如不要把起点或目标"
                        "取在直臂位形上），或加中间路点绕开";
      return attempt;
    }

    q_init = ik.q;
  }

  // ---- 3. 关节侧重建：用**更稀疏**的路点，段长取相邻路点时刻之差 ----
  //
  // 为什么不能把每个采样点都直接当路点（本任务最容易踩的坑）：
  //   buildQuinticTrajectory 会解一个线性系统，让路点处的 jerk 与 snap 连续，
  //   于是解出来的路点速度/加速度是由"路点位置的高阶差分"决定的：
  //       jerk 项 ~ jerk·T³、snap 项 ~ snap·T⁴
  //   而 T = sample_dt = 10 ms 时，这两个量分别是 1e-9 rad 与 1e-10 rad 量级，
  //   远小于逆解给出的 ~1e-6 rad 关节角噪声 —— 高阶信息完全被噪声淹没，解出来的
  //   加速度/jerk 就由噪声主导（实测关节加速度能虚高到 40+ rad/s²），而位置仍然
  //   插得很准。一句话：**10 ms 的采样间隔根本承载不了 C⁴ 连续条件**。
  // 把重建路点间距放宽到 rebuild_dt（默认 100 ms）后，噪声按 1/T³ 的放大倍数降到
  // 1/1000，重建轨迹的速度/加速度/jerk 才是物理量，也才谈得上拿去和关节限位比较。
  // （实测数据写在 TaskRequest::rebuild_dt 的注释里。）
  // 注意：采样与自检仍走细网格（sample_dt），所以路径保真度不受影响。
  const unsigned int stride =
    std::max(1U, static_cast<unsigned int>(std::lround(req.rebuild_dt / dt)));

  std::vector<KDL::JntArray> knots;
  std::vector<double> knot_times;
  for (unsigned int k = 0; k <= intervals; k += stride) {
    knots.push_back(joints[k]);
    knot_times.push_back(times[k]);
  }
  // 末点必须落在轨迹终点上（否则重建出的轨迹会提前停住）
  if (knot_times.back() < times.back() - 1e-12) {
    knots.push_back(joints[intervals]);
    knot_times.push_back(times[intervals]);
  }
  if (knots.size() < 2) {
    knots = { joints.front(), joints.back() };
    knot_times = { times.front(), times.back() };
  }

  std::vector<double> durations(knot_times.size() - 1, 0.0);
  for (std::size_t k = 0; k + 1 < knot_times.size(); ++k) {
    durations[k] = knot_times[k + 1] - knot_times[k];
  }

  const kdl_interpolation::TrajectoryResult build =
    kdl_interpolation::buildQuinticTrajectory(knots, durations, ctx.joint_limits);
  if (!build.success) {
    attempt.message = "关节侧重建失败：" + build.message +
                      "；请放慢（增大 duration）、减少中间路点，或放宽关节限位";
    return attempt;
  }
  attempt.trajectory = build.trajectory;
  attempt.rebuild_knots = static_cast<unsigned int>(knots.size());

  // ---- 4. 闭环自检：重建轨迹 → FK → 与笛卡尔参考比较 ----
  // 这才是"解算对不对"的客观判据：不是看中间量，而是看最终下发的关节轨迹
  // 正向运动学回去之后，还落不落在原来那条笛卡尔路径上。
  for (unsigned int k = 0; k <= intervals; ++k) {
    KDL::JntArray q;
    KDL::JntArray qdot;
    KDL::JntArray qddot;
    if (!attempt.trajectory.sample(times[k], q, qdot, qddot)) {
      attempt.message = "重建轨迹采样失败（t = " + num(times[k]) + " s）";
      return attempt;
    }
    KDL::Frame fk;
    if (!kdl_kinematics::forwardKinematics(ctx.chain, q, fk)) {
      attempt.message = "自检正运动学失败（t = " + num(times[k]) + " s）";
      return attempt;
    }
    attempt.position_error = std::max(attempt.position_error, (fk.p - references[k].p).Norm());
    attempt.orientation_error =
      std::max(attempt.orientation_error, rotationAngle(references[k].M, fk.M));
  }

  attempt.success = true;
  attempt.samples = intervals + 1;
  attempt.sample_dt = dt;
  return attempt;
}

}  // namespace

bool CartesianSpaceTask::estimateDuration(
  const RobotContext & ctx, const KDL::Frame & from, const KDL::Frame & to, double & duration,
  std::string & message)
{
  const double delta_position = (to.p - from.p).Norm();
  const double delta_angle = rotationAngle(from.M, to.M);

  const kdl_interpolation::CartesianLimits & limits = ctx.cartesian_limits;
  const bool any_limit = limits.check_linear_velocity() || limits.check_angular_velocity() ||
                         limits.check_linear_acceleration() ||
                         limits.check_angular_acceleration();
  if (!any_limit) {
    message = "自动定时失败：未设置任何笛卡尔限位（速度/加速度），无法反推时长；"
              "请显式给 duration/durations，或先设置 cartesian_limits";
    return false;
  }

  // 速度侧：T ≥ 1.875·Δ/v_max（位置）、T ≥ 1.875·θ/ω_max（姿态）
  // 加速度侧：T ≥ √(5.7735·Δ/a_max)、T ≥ √(5.7735·θ/α_max)
  // 四条下界取最大者。把加速度也算进来，可以少一次"自动定时后又被判为不可行"。
  double longest = 0.0;
  if (limits.check_linear_velocity()) {
    longest = std::max(longest, kPeakVelocityFactor * delta_position / limits.max_linear_velocity);
  }
  if (limits.check_angular_velocity()) {
    longest = std::max(longest, kPeakVelocityFactor * delta_angle / limits.max_angular_velocity);
  }
  if (limits.check_linear_acceleration()) {
    longest = std::max(
      longest, std::sqrt(kPeakAccelerationFactor * delta_position / limits.max_linear_acceleration));
  }
  if (limits.check_angular_acceleration()) {
    longest = std::max(
      longest, std::sqrt(kPeakAccelerationFactor * delta_angle / limits.max_angular_acceleration));
  }

  if (longest <= 0.0) {
    // 起点与终点位姿几乎相同：给一个最短保持段（理由同任务1）。
    duration = kMinSegmentDuration;
    return true;
  }

  duration = longest * kDurationSafetyFactor;
  return true;
}

bool CartesianSpaceTask::validate(
  const RobotContext & ctx, const TaskRequest & req, std::string & message) const
{
  // 段数：起点由本任务插入，所以单段时 = 1，给了路点时 = 路点数。
  const unsigned int segments =
    req.cartesian_waypoints.empty() ? 1U : static_cast<unsigned int>(req.cartesian_waypoints.size());

  if (!req.durations.empty()) {
    if (req.durations.size() != segments) {
      message = "durations 长度 " + std::to_string(req.durations.size()) + " 与段数 " +
                std::to_string(segments) +
                " 不一致（起点由任务插入：单段时段数为 1，给路点时等于路点数）";
      return false;
    }
    for (std::size_t k = 0; k < req.durations.size(); ++k) {
      if (req.durations[k] <= 0.0) {
        message = "durations[" + std::to_string(k) + "] 必须 > 0（收到 " +
                  num(req.durations[k]) + " s）";
        return false;
      }
    }
  }
  if (req.duration < 0.0) {
    message = "duration 必须 >= 0（0 表示自动估算，收到 " + num(req.duration) + " s）";
    return false;
  }
  if (req.sample_dt <= 0.0) {
    message = "sample_dt 必须 > 0（收到 " + num(req.sample_dt) + " s）";
    return false;
  }
  if (req.rebuild_dt <= 0.0) {
    message = "rebuild_dt 必须 > 0（收到 " + num(req.rebuild_dt) + " s）";
    return false;
  }
  if (req.joint_jump_threshold <= 0.0) {
    message = "joint_jump_threshold 必须 > 0（收到 " + num(req.joint_jump_threshold) + " rad）";
    return false;
  }
  if (req.position_tolerance <= 0.0 || req.orientation_tolerance <= 0.0) {
    message = "自检阈值必须 > 0（位置容差 " + num(req.position_tolerance) + " m，姿态容差 " +
              num(req.orientation_tolerance) + " rad）";
    return false;
  }

  // 需要自动定时时，先确认有限位可用，免得走到 solve() 才发现无从下手。
  const bool auto_duration = req.durations.empty() && req.duration <= 0.0;
  const kdl_interpolation::CartesianLimits & limits = ctx.cartesian_limits;
  if (auto_duration && !limits.check_linear_velocity() && !limits.check_angular_velocity() &&
      !limits.check_linear_acceleration() && !limits.check_angular_acceleration())
  {
    message = "未给时长且未设置任何笛卡尔限位，无法自动定时";
    return false;
  }

  return true;
}

ControlResult CartesianSpaceTask::solve(
  const RobotContext & ctx, const TaskRequest & req, const KDL::JntArray & q_now,
  const KDL::JntArray & qdot_now) const
{
  (void)qdot_now;  // 轨迹两端静止，当前速度不参与解算（见 TaskBase 注释）

  ControlResult result;
  result.type = TaskType::kCartesianSpace;

  // ---- 1. 起点位姿：默认用 FK(q_now)，保证"起点"与机器人真实状态一致 ----
  KDL::Frame start = req.start_pose;
  if (!req.has_start_pose) {
    if (!kdl_kinematics::forwardKinematics(ctx.chain, q_now, start)) {
      result.message = "无法由当前关节角计算起点位姿（正运动学失败）";
      return result;
    }
  }

  // ---- 2. 组路点：起点由本任务插入 ----
  std::vector<KDL::Frame> path;
  path.reserve(1 + req.cartesian_waypoints.size());
  path.push_back(start);
  if (req.cartesian_waypoints.empty()) {
    path.push_back(req.goal_pose);
  } else {
    path.insert(path.end(), req.cartesian_waypoints.begin(), req.cartesian_waypoints.end());
  }
  const unsigned int segments = static_cast<unsigned int>(path.size()) - 1U;

  // ---- 3. 逐段时长 ----
  std::vector<double> durations(segments, 0.0);
  if (!req.durations.empty()) {
    durations = req.durations;
  } else if (req.duration > 0.0) {
    for (unsigned int k = 0; k < segments; ++k) {
      durations[k] = req.duration;
    }
  } else {
    for (unsigned int k = 0; k < segments; ++k) {
      double estimate = 0.0;
      std::string reason;
      if (!estimateDuration(ctx, path[k], path[k + 1], estimate, reason)) {
        result.message = "第 " + std::to_string(k + 1) + " 段自动定时失败：" + reason;
        return result;
      }
      durations[k] = estimate;
    }
  }

  // ---- 4. 笛卡尔轨迹（位置五次 + 姿态 slerp），超限在这里就会被挡住 ----
  const kdl_interpolation::CartesianResult cartesian = kdl_interpolation::buildCartesianTrajectory(
    path, durations, req.behavior, ctx.cartesian_limits);
  if (!cartesian.success) {
    result.message = "构建笛卡尔轨迹失败：" + cartesian.message;
    return result;
  }

  // ---- 5. 离散 + IK + 重建 + 自检；自检超差就把采样步长减半重算一次 ----
  const double dt = req.sample_dt > 0.0 ? req.sample_dt : kDefaultSampleDt;
  Attempt attempt = runAttempt(ctx, req, cartesian.trajectory, q_now, dt);
  bool retried = false;
  if (attempt.success && (attempt.position_error > req.position_tolerance ||
                          attempt.orientation_error > req.orientation_tolerance))
  {
    retried = true;
    Attempt finer = runAttempt(ctx, req, cartesian.trajectory, q_now, dt * 0.5);
    if (finer.success) {
      // 即使更细的采样仍然超差，也保留它：误差数字更可信，便于判断该往哪儿调。
      attempt = finer;
    }
  }

  if (!attempt.success) {
    result.message = attempt.message;
    result.ik_failures = attempt.ik_failures;
    return result;
  }

  // 失败时也把轨迹填上：这类失败下轨迹本身是完整的（见 ControlResult 注释）。
  result.joint_trajectory = attempt.trajectory;
  result.duration = attempt.trajectory.duration();
  result.position_error = attempt.position_error;
  result.orientation_error = attempt.orientation_error;
  result.ik_failures = attempt.ik_failures;
  result.min_singular_value = attempt.min_singular_value;

  if (attempt.position_error > req.position_tolerance ||
      attempt.orientation_error > req.orientation_tolerance)
  {
    result.success = false;
    result.message = "重建关节轨迹的 FK 自检超差：位置残差峰值 " + num(attempt.position_error) +
                     " m、姿态残差峰值 " + num(attempt.orientation_error) + " rad（阈值 " +
                     num(req.position_tolerance) + " m / " + num(req.orientation_tolerance) +
                     " rad" + (retried ? "，已把 sample_dt 减半重算一次" : "") +
                     "）；建议增加中间路点或减小 sample_dt";
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace kdl_control
