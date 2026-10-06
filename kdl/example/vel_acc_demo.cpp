// Copyright (c) 2026, kdl_kinematics authors.
// 教学示例：速度级与加速度级的正/反向转换。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools vel_acc_demo
//
// 演示顺序：
//   1) 关节速度   -> 末端速度      twist = J·q̇
//   2) 末端速度   -> 关节速度      pinv 与 wdls 两种求逆 + 往返校验
//   3) 中间项     -> J̇·q̇          加速度级特有的科氏/离心项
//   4) 关节加速度 -> 末端加速度     a = J·q̈ + J̇·q̇
//   5) 末端加速度 -> 关节加速度     q̈ = J⁺·(a − J̇·q̇) + 往返校验
//   6) 奇异位形   -> σ_min 对比，以及 pinv / wdls 的表现差异
//
// 三个关键教学点：
//   a) 速度级正向是纯线性映射，永远成功；反向才需要伪逆，才可能奇异。
//   b) 加速度比速度多一个 J̇·q̇ 项，所以"关节不加速"不代表"末端不加速"。
//   c) 加速度逆解可以复用速度级求解器，因为扣掉 J̇·q̇ 后方程形式完全相同。

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/SVD>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_acceleration.hpp"
#include "kdl_jacobian.hpp"
#include "kdl_kinematics_print.hpp"
#include "kdl_tools.hpp"
#include "kdl_velocity.hpp"

// 由 CMake 在编译期传入（见 CMakeLists.txt 的 KDL_TOOLS_MODEL_DIR）。
#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

namespace
{

/**
 * @brief 计算两个关节向量之间的最大绝对偏差。
 * @param a 向量 a。
 * @param b 向量 b。
 * @return max|a(i) - b(i)|；两者长度取较小者。
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
 * @brief 计算关节速度向量中绝对值最大的分量，用来体现"解是否被放大"。
 * @param qdot 关节速度向量。
 * @return max|qdot(i)|。
 */
double maxAbsValue(const KDL::JntArray & qdot)
{
  double worst = 0.0;
  for (unsigned int i = 0; i < qdot.rows(); ++i) {
    worst = std::max(worst, std::abs(qdot(i)));
  }
  return worst;
}

/**
 * @brief 用给定关节角填充一个 JntArray（多余的自由度保持为 0）。
 */
KDL::JntArray makeJntArray(unsigned int n, const double * values, unsigned int count)
{
  KDL::JntArray array(n);
  for (unsigned int i = 0; i < n && i < count; ++i) {
    array(i) = values[i];
  }
  return array;
}

/**
 * @brief 取出 J(q) 的最小奇异值所对应的左奇异向量，即"最难运动"的笛卡尔方向。
 * @param chain 运动学链。
 * @param q     关节角，单位 rad。
 * @return 该方向的单位 Twist。
 *
 * @note 这个方向对应 σ_min：沿着它给末端速度指令时，
 *       纯伪逆需要把指令放大 1/σ_min 倍才能实现，因此最能暴露奇异的影响。
 * @note 本函数只服务于教学演示，所以留在示例里而没有进库。
 */
KDL::Twist minSingularDirection(const KDL::Chain & chain, const KDL::JntArray & q)
{
  KDL::Jacobian jacobian;
  kdl_kinematics::computeJacobian(chain, q, jacobian);

  const Eigen::MatrixXd matrix = jacobian.data;
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // U 的列与奇异值一一对应（Eigen 按降序给出），最后一列即 σ_min 对应的方向。
  const Eigen::VectorXd u = svd.matrixU().col(svd.matrixU().cols() - 1);
  return KDL::Twist(KDL::Vector(u(0), u(1), u(2)), KDL::Vector(u(3), u(4), u(5)));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vel_acc_demo");

  // ---- 0. 建链 ----
  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    RCLCPP_ERROR(node->get_logger(), "建链失败，退出");
    rclcpp::shutdown();
    return 1;
  }
  const unsigned int n = chain.getNrOfJoints();
  std::cout << "\nchain: " << n << " joints, " << chain.getNrOfSegments() << " segments\n";

  // ---- 输入：一组关节角、速度、加速度 ----
  const double q_values[] = {0.3, -0.4, 0.6, 0.2, -0.3, 0.5};      // rad
  const double qdot_values[] = {0.1, -0.2, 0.3, 0.0, 0.15, -0.25};  // rad/s
  const double qddot_values[] = {0.5, 0.4, -0.3, 0.2, -0.1, 0.1};   // rad/s²

  const KDL::JntArray q = makeJntArray(n, q_values, 6);
  const KDL::JntArray qdot = makeJntArray(n, qdot_values, 6);
  const KDL::JntArray qddot = makeJntArray(n, qddot_values, 6);

  std::cout << "\n输入:\n";
  std::cout << "  q     (rad)    ";  kdl_kinematics::printJntArray(q);
  std::cout << "  qdot  (rad/s)  ";  kdl_kinematics::printJntArray(qdot);
  std::cout << "  qddot (rad/s²) ";  kdl_kinematics::printJntArray(qddot);

  // ---- 1. 关节速度 -> 末端速度 ----
  std::cout << "\n>>> 1) 关节速度 -> 末端速度: twist = J(q)·qdot\n";
  KDL::Twist twist;
  if (!kdl_kinematics::jointToCartesianVel(chain, q, qdot, twist)) {
    rclcpp::shutdown();
    return 1;
  }
  kdl_kinematics::printTwist(twist);
  std::cout << "  (linear: m/s, angular: rad/s)\n";

  // ---- 2. 末端速度 -> 关节速度 ----
  std::cout << "\n>>> 2) 末端速度 -> 关节速度（两种求逆对比）\n";
  const kdl_kinematics::VelocityResult vel_pinv =
    kdl_kinematics::cartesianToJointVel(chain, q, twist);
  kdl_kinematics::printVelocityResult(vel_pinv);
  std::cout << "  与 qdot 的往返偏差(应≈0): " << maxAbsDiff(vel_pinv.qdot, qdot) << "\n";

  const kdl_kinematics::VelocityResult vel_wdls =
    kdl_kinematics::cartesianToJointVelDamped(chain, q, twist);
  kdl_kinematics::printVelocityResult(vel_wdls);
  std::cout << "  与 qdot 的往返偏差(应≈0): " << maxAbsDiff(vel_wdls.qdot, qdot) << "\n";
  std::cout << "  说明：位形不奇异时，两种求逆都能精确还原 qdot。\n";

  // ---- 3. J̇·q̇：加速度级特有的科氏/离心项 ----
  std::cout << "\n>>> 3) J̇·q̇（科氏/离心项，单位 m/s² 与 rad/s²）\n";
  KDL::Twist jac_dot_q_dot;
  if (!kdl_kinematics::jacobianDotTimesQdot(chain, q, qdot, jac_dot_q_dot)) {
    rclcpp::shutdown();
    return 1;
  }
  kdl_kinematics::printTwist(jac_dot_q_dot);

  // ---- 4. 关节加速度 -> 末端加速度 ----
  std::cout << "\n>>> 4) 关节加速度 -> 末端加速度: a = J·q̈ + J̇·q̇\n";
  KDL::Twist acceleration;
  if (!kdl_kinematics::jointToCartesianAcc(chain, q, qdot, qddot, acceleration)) {
    rclcpp::shutdown();
    return 1;
  }
  kdl_kinematics::printTwist(acceleration);
  std::cout << "  (linear: m/s², angular: rad/s²)\n";

  // 为了对比，单独挑出 J·q̈ 这一项的贡献：由 a − J̇·q̇ 反推即可，
  // 既等价又省一次矩阵乘法。
  const KDL::Twist only_j_q_ddot = acceleration - jac_dot_q_dot;
  std::cout << "  其中 J·q̈ 这一项 = a − J̇·q̇：\n";
  kdl_kinematics::printTwist(only_j_q_ddot);

  // ---- 5. 末端加速度 -> 关节加速度 ----
  std::cout << "\n>>> 5) 末端加速度 -> 关节加速度（两种求逆对比）\n";
  const kdl_kinematics::AccelerationResult acc_pinv =
    kdl_kinematics::cartesianToJointAcc(chain, q, qdot, acceleration);
  kdl_kinematics::printAccelerationResult(acc_pinv);
  std::cout << "  与 qddot 的往返偏差(应≈0): " << maxAbsDiff(acc_pinv.qddot, qddot) << "\n";

  const kdl_kinematics::AccelerationResult acc_wdls =
    kdl_kinematics::cartesianToJointAccDamped(chain, q, qdot, acceleration);
  kdl_kinematics::printAccelerationResult(acc_wdls);
  std::cout << "  与 qddot 的往返偏差(应≈0): " << maxAbsDiff(acc_wdls.qddot, qddot) << "\n";

  // ---- 6. 奇异位形：σ_min 与两种求逆的差异 ----
  // 全零位形比工作位形更接近奇异（σ_min 更小），用它放大伪逆与阻尼求逆的差异。
  std::cout << "\n>>> 6) 奇异位形对比\n";
  KDL::JntArray q_singular(n);  // 全零

  std::cout << "  σ_min(工作位形) = " << kdl_kinematics::minSingularValue(chain, q) << "\n";
  std::cout << "  σ_min(全零位形) = "
            << kdl_kinematics::minSingularValue(chain, q_singular) << "\n";
  std::cout << "  => 全零位形的 σ_min 小得多，更接近奇异。\n";

  // 关键：沿"最难运动"的方向给指令。若随便挑一个方向，可能刚好落在
  // 好方向上，三种求法结果都一样，看不出奇异的影响。
  const KDL::Twist probe = minSingularDirection(chain, q_singular) * 0.1;
  std::cout << "\n  探针速度（沿最难运动的方向，幅值 0.1）:\n";
  kdl_kinematics::printTwist(probe);

  const kdl_kinematics::VelocityResult r_pinv =
    kdl_kinematics::cartesianToJointVel(chain, q_singular, probe);

  // 显式传 1e-5（= KDL 原生默认 eps）以复现下面要讲的坑：
  // 此时 wdls 认为"没有奇异"，阻尼系数被算成 0，结果与 pinv 完全一致。
  const kdl_kinematics::VelocityResult r_wdls_strict =
    kdl_kinematics::cartesianToJointVelDamped(chain, q_singular, probe, 0.05, 1e-5);

  // 用函数默认的 singular_eps = 1e-2（与机器人尺度匹配），阻尼才真正生效。
  const kdl_kinematics::VelocityResult r_wdls =
    kdl_kinematics::cartesianToJointVelDamped(chain, q_singular, probe, 0.05);

  std::cout << "\n  同一个末端速度，三种求法得到的最大关节速度:\n";
  std::cout << "    pinv                         : " << maxAbsValue(r_pinv.qdot)
            << " rad/s\n";
  std::cout << "    wdls (singular_eps = 1e-5)   : " << maxAbsValue(r_wdls_strict.qdot)
            << " rad/s   <- 与 pinv 相同\n";
  std::cout << "    wdls (singular_eps = 1e-2)   : " << maxAbsValue(r_wdls.qdot)
            << " rad/s\n";

  std::cout << "\n  结论：\n"
            << "    1) σ_min 越小越接近奇异。纯伪逆会把末端速度放大约 1/σ_min 倍，\n"
            << "       得到物理上无法执行的关节速度。\n"
            << "    2) KDL 的 wdls 只在自判奇异（σ_min < eps）时才施加阻尼，而其原生\n"
            << "       默认 eps = 1e-5 过于严格，此时它与 pinv 结果完全相同，\n"
            << "       看起来像阻尼失效了，其实是根本没被触发。\n"
            << "    3) 把 singular_eps 放宽到与机器人尺度匹配后，阻尼才真正起作用，\n"
            << "       关节速度回到可执行范围。\n";

  rclcpp::shutdown();
  return 0;
}
