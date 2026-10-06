// Copyright (c) 2026, kdl_kinematics authors.
// 教学示例：4 轴臂的**解析逆解**（平面 2R + 2 自由度腕部），并做穷举往返校验。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools ik_analytic_demo
//
// 本示例演示四件事：
//   1) 从 URDF 建链 → 提取"平面 2R + 腕部"的几何参数（含结构判据校验）；
//   2) 解析逆解 solveIkAnalytic：由工具点位置 + 自转角求 4 个关节角；
//   3) **穷举往返校验**：FK → IK → FK，位置误差应在机器精度量级；
//   4) 与数值解 solveIkLma 并排对比，看清"4 维任务 vs 6 维任务"的差别。
//
// 为什么 4 轴臂要用解析解：它只有 4 个自由度，任务空间只有 4 维；拿 6 维位姿去要求
// 它是**超定**的，数值解只能给最小二乘意义下的近似，常常直接不收敛。解析解直接解
// 4 维问题，闭式、无种子依赖、一次给出全部解。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_fk.hpp"
#include "kdl_ik.hpp"
#include "kdl_ik_analytic.hpp"
#include "kdl_tools.hpp"

namespace
{

/// 由 CMake 在编译期传入（见 CMakeLists.txt 的 KDL_TOOLS_MODEL_DIR）。
#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

std::string resolveUrdfPath(rclcpp::Node & node)
{
  if (node.has_parameter("urdf_file")) {
    const std::string from_param = node.get_parameter("urdf_file").as_string();
    if (!from_param.empty()) {
      return from_param;
    }
  }
  return std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
}

/// 打印提取出来的几何量，便于和 URDF 里的数值对照。
void printGeometry(const kdl_kinematics::Planar4ArmGeometry & geo)
{
  std::printf("  joint1/joint2 公共轴 axis = [%+.6f %+.6f %+.6f]\n",
    geo.axis.x(), geo.axis.y(), geo.axis.z());
  std::printf("  joint1 轴上一点 origin   = [%+.6f %+.6f %+.6f]\n",
    geo.origin.x(), geo.origin.y(), geo.origin.z());
  std::printf("  臂平面内正交基 u = [%+.4f %+.4f %+.4f]  v = [%+.4f %+.4f %+.4f]\n",
    geo.u.x(), geo.u.y(), geo.u.z(), geo.v.x(), geo.v.y(), geo.v.z());
  std::printf("  平面内连杆长度 L1 = %.6f m   L2 = %.6f m\n", geo.L1, geo.L2);
  std::printf("  joint3 原点的平面外坐标  = %+.6f m\n", geo.out_of_plane);
  std::printf("  角度偏置 delta1 = %+.6f rad   delta2 = %+.6f rad\n", geo.delta1, geo.delta2);
  std::printf("  腕心距离 wrist_h = %.6f m   工具点离轴半径 |c⊥| = %.6f m\n",
    geo.wrist_h, geo.tool_perp_radius);
  std::printf("  c⊥ 在 q=0 处的方向 = [%+.6f %+.6f %+.6f]\n",
    geo.tool_perp_ref.x(), geo.tool_perp_ref.y(), geo.tool_perp_ref.z());
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ik_analytic_demo");

  const std::string urdf_file = resolveUrdfPath(*node);
  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    RCLCPP_ERROR(node->get_logger(), "建链失败，退出");
    rclcpp::shutdown();
    return 1;
  }
  const unsigned int n = chain.getNrOfJoints();
  std::printf("\nchain: %u joints, %u segments\n", n, chain.getNrOfSegments());

  // =====================================================================
  // 1) 几何提取
  // =====================================================================
  std::printf("\n>>> 1) 从链里提取几何参数（不硬编码任何连杆长度）\n");
  kdl_kinematics::Planar4ArmGeometry geo;
  std::string message;
  if (!kdl_kinematics::extractPlanar4ArmGeometry(chain, geo, message)) {
    std::printf("  提取失败：%s\n", message.c_str());
    std::printf("  说明：本示例只适用于「joint1/joint2 平行 + joint3 轴在臂平面内 +\n");
    std::printf("        joint3/joint4 轴正交」的构型。\n");
    rclcpp::shutdown();
    return 1;
  }
  printGeometry(geo);

  // =====================================================================
  // 2) 单点求解演示
  // =====================================================================
  std::printf("\n>>> 2) 单点求解：给定一组关节角，用 FK 造出工具点位置，再解回来\n");
  const double q_true_values[] = {0.5, -0.4, 0.6, 0.3};
  KDL::JntArray q_true(n);
  for (unsigned int i = 0; i < n; ++i) {
    q_true(i) = q_true_values[i];
  }
  KDL::Frame pose_true;
  if (!kdl_kinematics::forwardKinematics(chain, q_true, pose_true)) {
    std::printf("  FK 失败\n");
    rclcpp::shutdown();
    return 1;
  }
  std::printf("  真值 q      = [%.4f %.4f %.4f %.4f]\n",
    q_true(0), q_true(1), q_true(2), q_true(3));
  std::printf("  FK 工具点 p = [%+.6f %+.6f %+.6f]\n",
    pose_true.p.x(), pose_true.p.y(), pose_true.p.z());

  KDL::JntArray q_seed(n);  // 初值刻意取零位，和真值差很远
  for (unsigned int i = 0; i < n; ++i) {
    q_seed(i) = 0.0;
  }
  const auto solved =
    kdl_kinematics::solveIkAnalytic(chain, geo, pose_true.p, q_true(3), q_seed);
  if (!solved.success()) {
    std::printf("  解析解失败：%s\n", solved.message.c_str());
  } else {
    std::printf("  解析解 q    = [%.6f %.6f %.6f %.6f]（牛顿 %u 步）\n",
      solved.q(0), solved.q(1), solved.q(2), solved.q(3), solved.iterations);
    KDL::Frame pose_back;
    kdl_kinematics::forwardKinematics(chain, solved.q, pose_back);
    std::printf("  FK(解) p    = [%+.6f %+.6f %+.6f]   位置误差 = %.3e m\n",
      pose_back.p.x(), pose_back.p.y(), pose_back.p.z(),
      (pose_back.p - pose_true.p).Norm());

    // 滚转自洽性：makeRollFrame + rollAngle 是"把参考姿态投影成绕工具轴自转"这套
    // 机制的全部，所以它必须能把自己解出来的 q4 原样读回来。这一条不成立的话，
    // 笛卡尔任务里的姿态投影就是在自欺欺人。
    kdl_kinematics::RollFrame frame;
    if (kdl_kinematics::makeRollFrame(chain, solved.q, frame)) {
      const double roll = kdl_kinematics::rollAngle(pose_back.M, frame);
      std::printf("  工具轴(量出) = [%+.6f %+.6f %+.6f]\n",
        frame.axis.x(), frame.axis.y(), frame.axis.z());
      std::printf("  滚转自洽     rollAngle(FK(解)) = %+.9f rad，给定 q4 = %+.9f rad，差 %.3e rad\n",
        roll, q_true(3), std::fabs(roll - q_true(3)));
    } else {
      std::printf("  滚转自洽     失败：makeRollFrame 返回 false\n");
    }
  }

  // =====================================================================
  // 3) 穷举往返校验
  // =====================================================================
  std::printf("\n>>> 3) 穷举往返校验：随机采样 q → FK → IK → FK，统计位置误差\n");
  // 采样范围取各关节行程之内（见 URDF 的 <limit>）。
  const double lo[4] = {-1.4, -2.8, -3.0, -1.4};
  const double hi[4] = {1.4, 2.8, 3.0, 1.4};
  std::mt19937 rng(20261006);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  const unsigned int kTrials = 400;
  unsigned int ok = 0;
  unsigned int unreachable = 0;
  unsigned int other_fail = 0;
  double worst_pos = 0.0;
  double sum_pos = 0.0;
  double worst_roll = 0.0;
  unsigned int roll_fail = 0;
  unsigned int worst_iter = 0;

  for (unsigned int t = 0; t < kTrials; ++t) {
    KDL::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
      q(i) = lo[i] + (hi[i] - lo[i]) * uni(rng);
    }
    KDL::Frame pose;
    if (!kdl_kinematics::forwardKinematics(chain, q, pose)) {
      continue;
    }
    const auto r = kdl_kinematics::solveIkAnalytic(chain, geo, pose.p, q(3), q_seed);
    if (!r.success()) {
      if (r.error_code == KDL::SolverI::E_OUT_OF_RANGE) {
        ++unreachable;
      } else {
        ++other_fail;
      }
      continue;
    }
    KDL::Frame back;
    kdl_kinematics::forwardKinematics(chain, r.q, back);
    const double e = (back.p - pose.p).Norm();
    worst_pos = std::max(worst_pos, e);
    sum_pos += e;
    worst_iter = std::max(worst_iter, r.iterations);

    // 滚转也要能原样读回来（理由见第 2 节）。
    kdl_kinematics::RollFrame frame;
    if (!kdl_kinematics::makeRollFrame(chain, r.q, frame)) {
      ++roll_fail;
    } else {
      double d = kdl_kinematics::rollAngle(back.M, frame) - q(3);
      while (d > 3.14159265358979323846) {
        d -= 2.0 * 3.14159265358979323846;
      }
      while (d <= -3.14159265358979323846) {
        d += 2.0 * 3.14159265358979323846;
      }
      worst_roll = std::max(worst_roll, std::fabs(d));
    }
    ++ok;
  }
  std::printf("  采样 %u 组：成功 %u，不可达 %u，其它失败 %u\n",
    kTrials, ok, unreachable, other_fail);
  std::printf("  位置误差：最大 %.3e m，平均 %.3e m\n",
    worst_pos, ok ? sum_pos / ok : 0.0);
  std::printf("  滚转误差：最大 %.3e rad（makeRollFrame 失败 %u 次）\n",
    worst_roll, roll_fail);
  std::printf("  牛顿迭代步数：最多 %u 步\n", worst_iter);

  // =====================================================================
  // 4) 与数值解对比
  // =====================================================================
  std::printf("\n>>> 4) 与数值解 solveIkLma 对比（同一个工具点位置）\n");
  const auto lma = kdl_kinematics::solveIkLma(chain, q_seed, pose_true);
  std::printf("  解析解：error_code = %d（0 = 成功），任务空间 4 维（位置 + 自转角）\n",
    solved.error_code);
  std::printf("  LMA  ：error_code = %d，任务空间 6 维（完整位姿）\n", lma.error_code);
  if (!lma.success()) {
    std::printf("         LMA 失败原因：%s\n", lma.message.c_str());
    std::printf("         这是**超定**的直接后果：4 个关节要满足 6 维位姿，一般无解。\n");
  } else {
    std::printf("         LMA 解 q = [%.6f %.6f %.6f %.6f]\n",
      lma.q(0), lma.q(1), lma.q(2), lma.q(3));
    std::printf("         注意：上面这个目标位姿是 FK 造出来的，**恰好落在可达流形上**，\n");
    std::printf("         所以 LMA 也能解。真正的差别要在「姿态够不着」时看（见下）。\n");
  }

  // ---- 关键对照：把目标姿态拧一个够不着的角度 ----
  //
  // 这才是 4 轴臂用解析解的理由。绕一个与工具轴垂直的轴把目标姿态转 0.3 rad：
  //   · 工具点位置没变；
  //   · 姿态离开了可达流形（4 轴臂只能绕工具轴自转）。
  // 数值解要去逼那 6 维误差，梯度会消失；解析解只取"绕工具轴自转"那一维，照样解。
  std::printf("\n  关键对照：把目标姿态绕「垂直于工具轴的轴」转 0.3 rad（位置不变）\n");
  kdl_kinematics::RollFrame frame_true;
  if (kdl_kinematics::makeRollFrame(chain, q_true, frame_true)) {
    KDL::Vector tilt_axis = frame_true.axis * KDL::Vector(0.0, 0.0, 1.0);
    if (tilt_axis.Norm() < 1e-6) {
      tilt_axis = frame_true.axis * KDL::Vector(1.0, 0.0, 0.0);
    }
    tilt_axis.Normalize();

    KDL::Frame tilted = pose_true;
    tilted.M = KDL::Rotation::Rot(tilt_axis, 0.3) * pose_true.M;

    const auto lma_tilt = kdl_kinematics::solveIkLma(chain, q_seed, tilted);
    std::printf("    数值解 LMA ：error_code = %d%s\n", lma_tilt.error_code,
      lma_tilt.success() ? "" : "（失败）");
    if (!lma_tilt.success()) {
      std::printf("                 原因：%s\n", lma_tilt.message.c_str());
    }

    // 解析解这边要**按笛卡尔任务的实际做法**走三步（见 CartesianSpaceTask）：
    //   ① 位置原样送进去，自转角先随便给 —— 它不影响 q1..q3；
    //   ② 在解出的位形上量滚转参考系，把参考姿态投影成滚转角；
    //   ③ 用投影出来的滚转角重解（等价于直接覆盖 q4）。
    // 少了 ② 就是拿一个"随手给的"自转角去冒充参考姿态的要求，滚转残差会虚高。
    const auto pos_only =
      kdl_kinematics::solveIkAnalytic(chain, geo, tilted.p, q_true(3), q_seed);
    if (!pos_only.success()) {
      std::printf("    解析解     ：位置这一步就失败了（%s）\n", pos_only.message.c_str());
    } else {
      kdl_kinematics::RollFrame frame_sol;
      const bool have_frame = kdl_kinematics::makeRollFrame(chain, pos_only.q, frame_sol);
      const double roll_cmd =
        have_frame ? kdl_kinematics::rollAngle(tilted.M, frame_sol) : q_true(3);
      const auto ana_tilt =
        kdl_kinematics::solveIkAnalytic(chain, geo, tilted.p, roll_cmd, q_seed);
      std::printf("    解析解     ：error_code = %d%s\n", ana_tilt.error_code,
        ana_tilt.success() ? "" : "（失败）");
      if (ana_tilt.success()) {
        KDL::Frame back_tilt;
        kdl_kinematics::forwardKinematics(chain, ana_tilt.q, back_tilt);
        KDL::Vector axis;
        std::printf("                 位置误差 = %.3e m\n", (back_tilt.p - tilted.p).Norm());
        std::printf("                 投影出的滚转角指令 = %+.6f rad\n", roll_cmd);
        std::printf("                 滚转残差 = %.3e rad（该臂能控制的那一维，投影后应当≈0）\n",
          std::fabs(kdl_kinematics::rollAngle(tilted.M, frame_sol) -
                    kdl_kinematics::rollAngle(back_tilt.M, frame_sol)));
        std::printf("                 完整夹角 = %.3e rad（含够不着的工具轴指向，本来就降不下去）\n",
          (tilted.M.Inverse() * back_tilt.M).GetRotAngle(axis, 1e-8));
      }
    }
  }

  std::printf("\n示例结束。\n");
  rclcpp::shutdown();
  return 0;
}
