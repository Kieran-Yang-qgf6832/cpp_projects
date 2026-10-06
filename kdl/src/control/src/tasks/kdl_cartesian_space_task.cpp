// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务2（笛卡尔空间运动）的实现。

#include "tasks/kdl_cartesian_space_task.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "kdl_fk.hpp"
#include "kdl_ik.hpp"
#include "kdl_ik_analytic.hpp"
#include "kdl_velocity.hpp"

namespace kdl_control
{
namespace
{

/// 本文件日志用的 logger 名（与节点名一致，便于过滤）。
constexpr const char * kLogTag = "kdl_control";

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
 * @note 阈值是本机器人量出来的（4 轴臂，见 control_demo 第 4c 节打印）：
 *         零位（直臂，典型奇异位形）= 0.0185
 *         工作构型                    = 0.18 ~ 0.21
 *       取 0.01 是为了"只有真的贴到奇异面上才触发"。**但余量只有约 1.85 倍**
 *       （0.0185 / 0.01），比 6 轴臂当年宽松得多；换模型或换标定后应当重新量一遍，
 *       不要默认这个 0.01 还合适。
 * @note 为什么 6×4 雅可比的最小奇异值仍然是有意义的量：4 个关节只能张成 6 维速度
 *       空间里的 4 维，那"够不着的 2 维"落在**左**零空间里，而奇异值来自右奇异向量，
 *       不受影响。所以 σ_min = 0 依旧精确对应"4 列线性相关"即真实运动学奇异。
 * @note 为什么必须在这里挡一下：接近奇异时逆解在零空间方向几乎不改变末端位姿，
 *       于是逐点逆解可以"合法地"在零空间里跳来跳去（实测 10 ms 内跳 0.017 rad，
 *       而末端只动了 9e-5 m）。插值重建会把这些跳变放大成巨大的速度/加速度/jerk，
 *       最后表现为"轨迹不可行"这种看不出真因的报错。在这里如实说清楚，
 *       比让人去猜"为什么加速度虚高 40 rad/s²"要省事得多。
 */
constexpr double kSingularityThreshold = 0.01;

/**
 * @brief 关节限位校验的容差 [rad]。
 *
 * @note **为什么不设成 0**：MuJoCo 的 `<joint range>` 是**软约束**，关节撞上限位后
 *       会停在略微越界的位置。实测（启动时的自由落体撞限位之后）：
 *         joint1 =  1.57102（限位 ±1.5708，越界 2.2e-4）
 *         joint2 =  3.10198（限位 ±3.1，   越界 2.0e-3）
 *         joint3 = -3.14932（限位 ±3.14，  越界 9.3e-3）
 *       而笛卡尔任务的起点是 FK(q_now) —— **机器人当前就在哪儿，不是我们能决定的**。
 *       容差留小了，任务会在第 0 个采样点就拒掉机器人自己的当前状态：
 *
 *           解算失败：第 0 个采样点（t = 0 s）的逆解超出关节行程：
 *                     关节 2 = 3.10199 rad，行程 [-3.1, 3.1] rad
 *
 *       （这是实测踩到的，当时容差是 1e-3，不是假想。）
 *
 * @note **为什么是 5e-2**：这个数要在两类量之间划一条线，两侧都有实测支撑。
 *
 *         必须容忍（否则误判）：
 *           软约束越界           9.3e-3   （上表，机器人"停"在限位外）
 *           从限位起步的外向过冲  2.0e-2   （实测：起点 joint2 = 3.102 时，
 *                                            参考轨迹先荡到 3.120 再往回收）
 *         必须挡住（真超行程）：
 *           目标本身够不着       0.1 ~ 1 rad   （工具点要跑偏 7 cm ~ 0.7 m）
 *
 *       5e-2 比上界（2.0e-2）大 2.5 倍，比下界（1e-1）小 2 倍。落在中间，但它是个
 *       **折中而不是推导**：0.02~0.05 rad 这一段会被放行，那对应工具点偏 1.4~3.5 cm，
 *       靠 MuJoCo 的关节限位（真实硬件靠机械挡块）兜住，不会有实际危害。
 *
 * @note 真实硬件上编码器不会越过机械限位读数，这个容差只会更宽裕。
 * @note 如果将来它把一条**合法**轨迹拒了，报错信息里有具体关节与数值，可直接照方抓药。
 */
constexpr double kJointLimitTolerance = 5e-2;

/// 把数值格式化成可读字符串（只用于拼错误信息）。
std::string num(double value, int precision = 6)
{
  std::ostringstream os;
  os << std::setprecision(precision) << value;
  return os.str();
}

/**
 * @brief 关节角是否落在行程内（含容差）；不在时把原因写进 message。
 *
 * @param ctx     [in]  机器人上下文（取 q_min/q_max 与关节数）。
 * @param q       [in]  待检查的关节角。
 * @param where   [in]  出错位置的前缀，例如"第 3 个采样点（t = 0.02 s）的逆解"。
 * @param hint    [in]  追在后面的建议。
 * @param message [out] 失败原因（中文）。
 * @return true 表示在行程内。
 *
 * @note 为什么不 clamp 而是判失败：夹到边界上会让"实际去的地方"和"要求去的地方"
 *       不一致，而且夹过之后末端根本不在目标位置——那是一条看起来成功、实际跑偏的
 *       轨迹。越界是调用者的意图问题，如实拒绝比悄悄修正安全。（与
 *       kdl_joint_space_task.cpp 对目标关节角的处理口径一致。）
 * @note q_min/q_max 没装配齐全时不判（与 RobotContext::valid() 的口径一致）。
 */
bool checkJointLimits(
  const RobotContext & ctx, const KDL::JntArray & q, const std::string & where,
  const std::string & hint, std::string & message)
{
  const unsigned int n = ctx.chain.getNrOfJoints();
  if (ctx.q_min.rows() != n || ctx.q_max.rows() != n) {
    return true;
  }
  for (unsigned int i = 0; i < n; ++i) {
    if (q(i) < ctx.q_min(i) - kJointLimitTolerance ||
        q(i) > ctx.q_max(i) + kJointLimitTolerance)
    {
      message = where + "超出关节行程：关节 " + std::to_string(i + 1) + " = " + num(q(i)) +
                " rad，行程 [" + num(ctx.q_min(i)) + ", " + num(ctx.q_max(i)) + "] rad" + hint;
      return false;
    }
  }
  return true;
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
 * @brief 把角度折到 (-π, π]。
 */
double wrapToPi(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
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
  /**
   * @brief 参考姿态与实到姿态的完整夹角峰值 [rad]（仅 4 轴解析解模式）。
   *
   * @note 这个量**故意不参与通过与否的判定**：4 轴臂够不着参考姿态里"工具轴指向"
   *       那一维，所以它天然不会小。它只回答一个诊断问题——"我们要求的姿态离这个臂
   *       真正能到的姿态流形有多远"。真要小，只能改目标，不能改控制器。
   */
  double pose_residual = 0.0;
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
 * @param geo       [in] 解析逆解用的几何；nullptr 或无效时退回通用 6 维数值解。
 * @return Attempt；失败时 message 说明原因。
 */
Attempt runAttempt(
  const RobotContext & ctx, const TaskRequest & req,
  const kdl_interpolation::CartesianTrajectory & cartesian, const KDL::JntArray & q_now,
  double sample_dt, const kdl_kinematics::Planar4ArmGeometry * geo)
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
  //
  // 4 轴臂走解析解：任务空间只有 4 维（工具点位置 3 维 + 绕工具轴自转 1 维）。参考
  // 位姿里"工具轴指向"那一维**不参与求解**——它由位置和肘部分支唯一决定；剩下的一维
  // （自转）从参考位姿里投影出来。
  //
  // 为什么必须这样：参考位姿是"位置五次 + 姿态 slerp"造出来的，而 4 轴臂在任意位置
  // 能实现的姿态只有绕工具轴自转的那 1 维，slerp 走出来的姿态路径几乎处处落在这个
  // 流形之外。拿 6 维误差去逼它，误差里有 2 维永远降不下去，梯度必然消失——实测
  // solveIkLma 在第 11/17/23/27 个采样点报 "The gradient of E towards the joints is
  // to small"，而第 0 点（正好是 FK(q_now)，可达）是好的。这是构型决定的，改参数、
  // 改路点、改初值都救不回来。详见头文件"任务空间是 4 维"一节。
  const bool analytic = (geo != nullptr && geo->valid);
  std::vector<KDL::JntArray> joints(intervals + 1);
  KDL::JntArray q_init = q_now;
  for (unsigned int k = 0; k <= intervals; ++k) {
    KDL::JntArray q_k(n);
    std::string failure;

    if (analytic) {
      // ① 位置原样用；自转先塞上一时刻的值（它不影响 q1..q3，随后会被覆盖）
      kdl_kinematics::AnalyticIkResult ik =
        kdl_kinematics::solveIkAnalytic(ctx.chain, *geo, references[k].p, q_init(3), q_init);
      if (!ik.success()) {
        failure = "解析逆解失败：" + ik.message + "；目标位置可能超出工作空间";
      } else {
        // ② 把参考姿态投影成"绕工具轴自转"这一维
        kdl_kinematics::RollFrame frame;
        if (!kdl_kinematics::makeRollFrame(ctx.chain, ik.q, frame)) {
          failure = "无法建立滚转参考系（第 4 个关节轴退化？）";
        } else {
          ik.q(3) = kdl_kinematics::rollAngle(references[k].M, frame);
          q_k = ik.q;
        }
      }
    } else {
      const kdl_kinematics::IkResult ik =
        kdl_kinematics::solveIkLma(ctx.chain, q_init, references[k], kIkEps, kIkMaxIter);
      if (!ik.success()) {
        failure = "IK 未收敛：" + ik.message + "；目标位姿可能超出工作空间";
      } else {
        q_k = ik.q;
      }
    }

    if (!failure.empty()) {
      ++attempt.ik_failures;
      attempt.message =
        "第 " + std::to_string(k) + " 个采样点（t = " + num(times[k]) + " s）" + failure;
      return attempt;
    }

    joints[k] = q_k;

    // 关节限位：解析解与数值解都刻意不做限位检查（"解出来是什么就是什么"，见
    // kdl_ik_analytic.hpp），责任在这里。这条检查在 4 轴臂上不是可选项 ——
    // joint1/joint4 行程只有 ±1.57 rad，很容易撞。
    if (!checkJointLimits(
          ctx, q_k, "第 " + std::to_string(k) + " 个采样点（t = " + num(times[k]) + " s）的逆解",
          "；请改目标位置，或放宽该关节的 URDF 限位", attempt.message))
    {
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

    q_init = q_k;
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
  //
  // 4 轴臂上"落在路径上"要分两半看：位置是硬约束（必须对上）；姿态里只有绕工具轴
  // 自转那一维是硬约束。参考位姿里工具轴指向那一维本来就够不着，把它算进判据会让
  // **任何**轨迹都判失败（误差恒为"目标姿态到可达流形的距离"），所以它单独记进
  // pose_residual，只做诊断、不参与通过与否。
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

    // 重建出的关节轨迹也要查一遍：五次插值在路点之间会过冲，而且**过冲量不小**。
    // 实测（control_demo 4a 那条轨迹）：joint2 从 -0.6 出发，轨迹峰值到了 -0.6748，
    // 过冲 0.075 rad ≈ 4.3°；joint3 过冲 3.8e-5 rad。也就是说这条检查**不是**锦上添花：
    // 只查逆解的话，一个"解出来刚好贴着限位"的关节会被插值推出限位而没人发现。
    // 上面查的是**逆解**，这里查的是**真正要下发的那条轨迹**，两者都要过。
    if (!checkJointLimits(
          ctx, q, "重建出的关节轨迹在 t = " + num(times[k]) + " s 处",
          "（插值过冲；请放慢轨迹或加中间路点）", attempt.message))
    {
      return attempt;
    }

    if (analytic) {
      // 滚转参考系取**实到位形**的：滚转角必须相对于同一个工具轴去比，否则
      // "参考姿态绕它自己的轴转了多少"和"实到姿态绕实到的轴转了多少"不是一回事。
      kdl_kinematics::RollFrame frame;
      if (!kdl_kinematics::makeRollFrame(ctx.chain, q, frame)) {
        attempt.message = "自检无法建立滚转参考系（t = " + num(times[k]) + " s）";
        return attempt;
      }
      const double roll_error =
        wrapToPi(
          kdl_kinematics::rollAngle(fk.M, frame) -
          kdl_kinematics::rollAngle(references[k].M, frame));
      attempt.orientation_error = std::max(attempt.orientation_error, std::abs(roll_error));
      attempt.pose_residual =
        std::max(attempt.pose_residual, rotationAngle(references[k].M, fk.M));
    } else {
      attempt.orientation_error =
        std::max(attempt.orientation_error, rotationAngle(references[k].M, fk.M));
    }
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

  // ---- 5. 逆解模式：4 轴臂走解析解（4 维任务空间），其它链退回通用数值解 ----
  //
  // 几何量只提取一次（沿轨迹要逐点调用几百次逆解），所以放在循环外面。
  // 提取失败说明这条链不是"平面 2R + 2 自由度腕部"构型，本任务的 4 维降维对它不成立，
  // 于是退回原来那套 6 维数值解 —— 行为与加这个分支之前完全一致，不会更差。
  // 但这件事必须说出来：静默降级正是这个仓库反复踩过的坑（见 kdl_ik_analytic.hpp 里
  // "把几何写死在代码里，换模型时不会报错、只会静默给出错解"那段）。
  kdl_kinematics::Planar4ArmGeometry geo;
  std::string geo_message;
  const bool analytic = kdl_kinematics::extractPlanar4ArmGeometry(ctx.chain, geo, geo_message);
  if (!analytic) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger(kLogTag),
      "笛卡尔任务：本运动学链不是「平面 2R + 2 自由度腕部」构型（%s），"
      "退回通用 6 维数值逆解。对自由度少于 6 的臂，除非目标恰好落在可达流形上，"
      "否则会解不出来。",
      geo_message.c_str());
  }

  // ---- 6. 离散 + IK + 重建 + 自检；自检超差就把采样步长减半重算一次 ----
  const double dt = req.sample_dt > 0.0 ? req.sample_dt : kDefaultSampleDt;
  Attempt attempt = runAttempt(
    ctx, req, cartesian.trajectory, q_now, dt, analytic ? &geo : nullptr);
  bool retried = false;
  if (attempt.success && (attempt.position_error > req.position_tolerance ||
                          attempt.orientation_error > req.orientation_tolerance))
  {
    retried = true;
    Attempt finer = runAttempt(
      ctx, req, cartesian.trajectory, q_now, dt * 0.5, analytic ? &geo : nullptr);
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
  result.pose_residual = attempt.pose_residual;
  result.ik_failures = attempt.ik_failures;
  result.min_singular_value = attempt.min_singular_value;

  if (attempt.position_error > req.position_tolerance ||
      attempt.orientation_error > req.orientation_tolerance)
  {
    result.success = false;
    result.message =
      "重建关节轨迹的 FK 自检超差：位置残差峰值 " + num(attempt.position_error) + " m、" +
      (analytic ? "绕工具轴滚转残差峰值 " : "姿态残差峰值 ") + num(attempt.orientation_error) +
      " rad（阈值 " + num(req.position_tolerance) + " m / " + num(req.orientation_tolerance) +
      " rad" + (retried ? "，已把 sample_dt 减半重算一次" : "") +
      "）；建议增加中间路点或减小 sample_dt";
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace kdl_control
