// Copyright (c) 2026, kdl_interpolation authors.
// 教学示例：关节空间五次多项式插值 —— 单段、多段 C⁴ 拼接、以及约束校验。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools quintic_demo
//
// 演示顺序：
//   1) 单段五次插值：6 个边界条件 → 6 个系数 → 采样，并验证边界条件被精确满足
//   2) 端点 jerk 的解析值 60·|Δq|/T³：为什么"想快，抖动就必然变大"
//   3) 多段 C⁴ 拼接：4 个路点 + 3 段时长 → 全局求解 → 验证 jerk/snap 连续
//   4) 约束校验：同一批路点、时间越短越容易判"不可行"（速度先超限）
//   5) 把可行的那条轨迹采样成 CSV，方便导入 MATLAB/Python 画曲线
//
// 四个关键教学点：
//   a) 五次多项式有 6 个系数，所以要 6 个边界条件；三次多项式管不住加速度。
//   b) 多段拼接时，只要"每个路点只有一组 (v, a)"，C¹/C² 连续就免费得到；
//      剩下的自由度用 jerk/snap 连续补齐，方程数恰好等于未知量数（2(m−1)）。
//   c) 两端 v=a=0 与两端 jerk=0 不可兼得：端点 jerk 由 ±60Δq/T³ 量级决定，
//      所以"可行不可行"必须靠校验 jerk 来判断，而不是靠把结果裁一裁。
//   d) 本示例不依赖 URDF / 机器人模型：插值是纯关节空间的事，给几个数组就能跑。

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_interpolation_print.hpp"
#include "kdl_quintic.hpp"

namespace
{

/// 演示用的关节数（本示例自包含，不读 URDF，取值与具体模型无关）。
constexpr unsigned int kNumJoints = 6;

/**
 * @brief 由固定长度数组构造一个关节向量。
 * @param values 关节角（或速度、加速度）数值。
 * @return 对应的 KDL::JntArray。
 */
KDL::JntArray makeJoints(const std::array<double, kNumJoints> & values)
{
  KDL::JntArray joints(kNumJoints);
  for (unsigned int i = 0; i < kNumJoints; ++i) {
    joints(i) = values[i];
  }
  return joints;
}

/**
 * @brief 计算两个关节向量的最大绝对偏差。
 * @param a 向量 a。
 * @param b 向量 b。
 * @return max|a(i) − b(i)|，长度取较小者。
 */
double maxAbsDiff(const KDL::JntArray & a, const KDL::JntArray & b)
{
  const unsigned int n = std::min(a.rows(), b.rows());
  double worst = 0.0;
  for (unsigned int i = 0; i < n; ++i) {
    worst = std::max(worst, std::abs(a(i) - b(i)));
  }
  return worst;
}

/**
 * @brief 取内部路点处相邻两段的 jerk 差值（C³ 连续的残差）。
 * @param trajectory [in] 轨迹。
 * @param knot       [in] 路点序号（内部路点，即 1 .. 段数−1）。
 * @param joint      [in] 关节序号。
 * @return 左段右端 jerk − 右段左端 jerk，理论值为 0。
 *
 * @note 这里刻意取出两段的系数各自求值，而不是在路点两侧各采一个点再相减：
 *       前者是"精确比较"，后者会混进 1e-9 的采样位置误差。
 */
double knotJerkGap(const kdl_interpolation::QuinticTrajectory & trajectory, unsigned int knot,
                   unsigned int joint)
{
  const kdl_interpolation::QuinticCoefficients & left =
    trajectory.segmentCoefficients(knot - 1, joint);
  const kdl_interpolation::QuinticCoefficients & right =
    trajectory.segmentCoefficients(knot, joint);
  return kdl_interpolation::quinticSegmentJerk(left, 1.0) -
         kdl_interpolation::quinticSegmentJerk(right, 0.0);
}

/**
 * @brief 取内部路点处相邻两段的 snap 差值（C⁴ 连续的残差）。
 * @param trajectory [in] 轨迹。
 * @param knot       [in] 路点序号（内部路点）。
 * @param joint      [in] 关节序号。
 * @return 左段右端 snap − 右段左端 snap，理论值为 0。
 */
double knotSnapGap(const kdl_interpolation::QuinticTrajectory & trajectory, unsigned int knot,
                   unsigned int joint)
{
  const kdl_interpolation::QuinticCoefficients & left =
    trajectory.segmentCoefficients(knot - 1, joint);
  const kdl_interpolation::QuinticCoefficients & right =
    trajectory.segmentCoefficients(knot, joint);
  return kdl_interpolation::quinticSegmentSnap(left, 1.0) -
         kdl_interpolation::quinticSegmentSnap(right, 0.0);
}

/**
 * @brief 把一条轨迹按固定步长采样写成 CSV（含 pos / vel / acc / jerk / snap 五组列）。
 * @param trajectory [in] 待导出的轨迹，须 valid()。
 * @param path       [in] 输出文件路径。
 * @return true 表示写入成功。
 *
 * @note 表头列名是 q0..qN / qdot0..qdotN / ... 这种"前缀 + 关节号"的形式，
 *       画图脚本 src/Interpolation/scripts/plot_quintic.py 依赖这个约定。
 */
bool writeCsv(const kdl_interpolation::QuinticTrajectory & trajectory, const std::string & path)
{
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }

  const unsigned int joints = trajectory.joints();
  csv << "t";
  for (unsigned int j = 0; j < joints; ++j) csv << ",q" << j;
  for (unsigned int j = 0; j < joints; ++j) csv << ",qdot" << j;
  for (unsigned int j = 0; j < joints; ++j) csv << ",qddot" << j;
  for (unsigned int j = 0; j < joints; ++j) csv << ",jerk" << j;
  for (unsigned int j = 0; j < joints; ++j) csv << ",snap" << j;
  csv << "\n";

  KDL::JntArray q, qdot, qddot, jerk, snap;
  const unsigned int steps = 400;
  for (unsigned int i = 0; i <= steps; ++i) {
    const double t = trajectory.duration() * static_cast<double>(i) / static_cast<double>(steps);
    trajectory.sample(t, q, qdot, qddot);
    trajectory.sampleDerivatives(t, jerk, snap);

    csv << t;
    for (unsigned int j = 0; j < joints; ++j) csv << "," << q(j);
    for (unsigned int j = 0; j < joints; ++j) csv << "," << qdot(j);
    for (unsigned int j = 0; j < joints; ++j) csv << "," << qddot(j);
    for (unsigned int j = 0; j < joints; ++j) csv << "," << jerk(j);
    for (unsigned int j = 0; j < joints; ++j) csv << "," << snap(j);
    csv << "\n";
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("quintic_demo");
  std::cout << std::setprecision(6);

  // =========================================================================
  // 1) 单段五次多项式：6 个边界条件 → 6 个系数
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 1) 单段五次多项式插值\n"
            << "=============================================================\n";

  const std::array<double, kNumJoints> q_start_values{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, kNumJoints> q_goal_values{0.8, -0.5, 0.6, -0.3, 0.4, 0.9};
  const double single_duration = 1.0;  // s

  std::cout << "  起点 q      = ";
  kdl_interpolation::printJointVector(makeJoints(q_start_values));
  std::cout << "  终点 q      = ";
  kdl_interpolation::printJointVector(makeJoints(q_goal_values));
  std::cout << "  两端 qdot = 0, qddot = 0（静止起步、静止停住）\n";
  std::cout << "  段时长 T    = " << single_duration << " s\n";

  // 六个边界条件全部显式传入 —— 这就是"单段便捷入口"的用法。
  // 每个关节各求一次，得到的系数彼此独立（关节之间没有耦合）。
  std::vector<kdl_interpolation::QuinticCoefficients> single_segments(kNumJoints);
  for (unsigned int j = 0; j < kNumJoints; ++j) {
    kdl_interpolation::computeQuinticCoefficients(
      q_start_values[j], 0.0, 0.0, q_goal_values[j], 0.0, 0.0, single_duration,
      single_segments[j]);
  }

  std::cout << "\n  关节 0 的系数（注意：是对 τ = t/T 的，不是对 t 的）\n";
  kdl_interpolation::printQuinticCoefficients(single_segments[0]);

  // 采样表：只挑关节 0、1 两列，避免表格宽到看不下去。
  std::cout << "\n  采样表（关节 0 | 关节 1，速度单位 rad/s，加速度单位 rad/s²）\n";
  std::cout << "      t[s]        q0        qd0        qdd0  |         q1        qd1        qdd1\n";
  for (unsigned int i = 0; i <= 10; ++i) {
    const double tau = static_cast<double>(i) / 10.0;
    const double t = tau * single_duration;

    double q0 = 0.0, qd0 = 0.0, qdd0 = 0.0;
    double q1 = 0.0, qd1 = 0.0, qdd1 = 0.0;
    kdl_interpolation::evaluateQuinticSegment(single_segments[0], tau, q0, qd0, qdd0);
    kdl_interpolation::evaluateQuinticSegment(single_segments[1], tau, q1, qd1, qdd1);

    std::cout << std::fixed << std::setprecision(4) << std::setw(10) << t << std::setw(11) << q0
              << std::setw(11) << qd0 << std::setw(11) << qdd0 << "  |" << std::setw(11) << q1
              << std::setw(11) << qd1 << std::setw(11) << qdd1 << "\n"
              << std::defaultfloat << std::setprecision(6);
  }

  // 边界条件校验：把 τ=0 与 τ=1 代回去，看六个条件是否被精确满足。
  double boundary_error = 0.0;
  for (unsigned int j = 0; j < kNumJoints; ++j) {
    double q = 0.0, qd = 0.0, qdd = 0.0;
    kdl_interpolation::evaluateQuinticSegment(single_segments[j], 0.0, q, qd, qdd);
    boundary_error = std::max(boundary_error, std::abs(q - q_start_values[j]));
    boundary_error = std::max(boundary_error, std::abs(qd));
    boundary_error = std::max(boundary_error, std::abs(qdd));

    kdl_interpolation::evaluateQuinticSegment(single_segments[j], 1.0, q, qd, qdd);
    boundary_error = std::max(boundary_error, std::abs(q - q_goal_values[j]));
    boundary_error = std::max(boundary_error, std::abs(qd));
    boundary_error = std::max(boundary_error, std::abs(qdd));
  }
  std::cout << "\n  六个边界条件的最大残差（应≈0）: " << boundary_error << "\n";
  std::cout << "  说明：曲线在两端不仅位置对得上，速度和加速度也精确为 0，\n"
            << "        所以它不会「起步一顿」或「到点急停」。\n";

  // =========================================================================
  // 2) 端点 jerk = 60·|Δq|/T³：想快，抖动必然变大
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 2) 端点 jerk = 60·|Δq| / T³（单段、两端 v=a=0 时的解析值）\n"
            << "=============================================================\n";
  std::cout << "  关节 0 从 " << q_start_values[0] << " 走到 " << q_goal_values[0] << " rad，"
            << "Δq = " << (q_goal_values[0] - q_start_values[0]) << " rad\n\n";
  std::cout << "      T[s]      理论 jerk[rad/s³]      起点实际       终点实际\n";

  for (double duration : {1.0, 0.5, 0.25}) {
    kdl_interpolation::QuinticCoefficients coefficients;
    kdl_interpolation::computeQuinticCoefficients(
      q_start_values[0], 0.0, 0.0, q_goal_values[0], 0.0, 0.0, duration, coefficients);

    const double analytic =
      60.0 * std::abs(q_goal_values[0] - q_start_values[0]) / (duration * duration * duration);
    const double jerk_start = kdl_interpolation::quinticSegmentJerk(coefficients, 0.0);
    const double jerk_end = kdl_interpolation::quinticSegmentJerk(coefficients, 1.0);

    std::cout << std::fixed << std::setprecision(4) << std::setw(10) << duration
              << std::setw(22) << analytic << std::setw(16) << jerk_start << std::setw(16)
              << jerk_end << "\n"
              << std::defaultfloat << std::setprecision(6);
  }
  std::cout << "\n  结论：T 减半，端点 jerk 变成 8 倍（因为∝1/T³）。\n"
            << "        这两行数字完全由「位移 ÷ 时间」决定，不是实现细节能改变的；\n"
            << "        想同时把 jerk 压下来，唯一办法是放慢或改路点。\n";

  // =========================================================================
  // 3) 多段 C⁴ 拼接：4 个路点 + 3 段时长 → 全局求解
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 3) 多段 C⁴ 拼接（4 个路点、3 段）\n"
            << "=============================================================\n";

  std::vector<KDL::JntArray> waypoints;
  waypoints.push_back(makeJoints({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  waypoints.push_back(makeJoints({0.5, -0.3, 0.4, 0.2, 0.3, -0.4}));
  waypoints.push_back(makeJoints({0.9, 0.2, -0.2, -0.4, 0.1, 0.5}));
  waypoints.push_back(makeJoints({0.3, 0.6, 0.5, 0.3, 0.2, 0.1}));

  const std::array<double, 3> base_durations{0.6, 0.5, 0.8};  // s，逐段显式给出

  // ---- 3a. 先不加任何约束，看纯插值结果 ----
  const std::vector<double> durations_plain(base_durations.begin(), base_durations.end());
  const kdl_interpolation::TrajectoryResult plain =
    kdl_interpolation::buildQuinticTrajectory(waypoints, durations_plain);

  kdl_interpolation::printTrajectoryResult(plain);
  if (!plain.success) {
    RCLCPP_ERROR(node->get_logger(), "示例无法继续：%s", plain.message.c_str());
    rclcpp::shutdown();
    return 1;
  }
  kdl_interpolation::printTrajectorySummary(plain.trajectory);
  kdl_interpolation::printKnotTable(plain.trajectory);

  // ---- 3b. 验证拼接处的 jerk（C³）与 snap（C⁴）连续 ----
  double jerk_gap = 0.0;
  double snap_gap = 0.0;
  for (unsigned int knot = 1; knot + 1 < waypoints.size(); ++knot) {
    for (unsigned int j = 0; j < kNumJoints; ++j) {
      jerk_gap = std::max(jerk_gap, std::abs(knotJerkGap(plain.trajectory, knot, j)));
      snap_gap = std::max(snap_gap, std::abs(knotSnapGap(plain.trajectory, knot, j)));
    }
  }
  std::cout << "\n  拼接处连续性残差（应≈0）:\n"
            << "    max|jerk 左右之差| = " << jerk_gap << " rad/s³\n"
            << "    max|snap 左右之差| = " << snap_gap << " rad/s⁴\n"
            << "  说明：速度、加速度在路点处本就用同一组值，所以天然连续；\n"
            << "        jerk 与 snap 的连续是「解出来的」——正是未知量数 = 方程数的结果。\n";

  // =========================================================================
  // 4) 约束校验：同一批路点，时间越短越容易判"不可行"
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 4) 约束校验：不可行就如实报错，不做 clamp\n"
            << "=============================================================\n";

  kdl_interpolation::JointLimits limits;
  limits.max_velocity = makeJoints({5.0, 5.0, 5.0, 5.0, 5.0, 5.0});         // rad/s
  limits.max_acceleration = makeJoints({30.0, 30.0, 30.0, 30.0, 30.0, 30.0});  // rad/s²
  limits.max_jerk = makeJoints({1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0});  // rad/s³
  kdl_interpolation::printJointLimits(limits);

  kdl_interpolation::TrajectoryResult feasible;
  bool has_feasible = false;
  for (double scale : {0.25, 0.5, 1.0}) {
    std::vector<double> durations;
    for (double base : base_durations) {
      durations.push_back(base * scale);
    }

    std::cout << "\n  >>> 时间缩放 " << scale << " → 各段时长 [s]: [";
    for (std::size_t k = 0; k < durations.size(); ++k) {
      std::cout << durations[k] << (k + 1 < durations.size() ? ", " : "");
    }
    std::cout << "]\n";

    const kdl_interpolation::TrajectoryResult result =
      kdl_interpolation::buildQuinticTrajectory(waypoints, durations, limits);
    kdl_interpolation::printTrajectoryResult(result);

    if (result.success && !has_feasible) {
      feasible = result;
      has_feasible = true;
    }
  }
  std::cout << "\n  结论：路点不变、只把时间压紧，峰值速度∝1/T、加速度∝1/T²、\n"
            << "        jerk∝1/T³，很快就撞上限。工程上正确的做法就是这里的做法——\n"
            << "        把「不可行」如实返回，让调用者决定放慢、改路点还是换方案。\n";

  // =========================================================================
  // 5) 导出 CSV
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 5) 导出 CSV\n"
            << "=============================================================\n";

  if (!has_feasible) {
    std::cout << "  没有可行的轨迹可导出，跳过。\n";
  } else {
    const std::string path = std::filesystem::absolute("quintic_trajectory.csv").string();
    if (writeCsv(feasible.trajectory, path)) {
      std::cout << "  已写入: " << path << "\n"
                << "  列含义: t, q0..q5, qdot0..qdot5, qddot0..qddot5, jerk0..jerk5, snap0..snap5\n"
                << "  401 行（t = 0 .. 总时长）等间隔采样。\n"
                << "  画图:   ros2 run kdl_tools plot_quintic " << path << "\n";
    } else {
      std::cout << "  写入失败: " << path << "\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}
