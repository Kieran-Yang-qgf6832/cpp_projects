// Copyright (c) 2026, kdl_dynamics authors.
// 教学示例：机器人动力学的逆解与正解。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools dynamics_demo
//
// 演示顺序：
//   1) 输入一组 (q, q̇, q̈)
//   2) 逆动力学  τ = ID(q, q̇, q̈)                        （ChainIdSolver_RNE）
//   3) 把 τ 拆成三项 τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)，      （ChainDynParam）
//      并检查 M 的对称性与正定性，最后与第 2 步的结果对照
//   4) 正动力学  q̈ = FD(q, q̇, τ)，再做 FD(ID(q̈)) 往返闭合 （ChainFdSolver_RNE）
//   5) 末端施加外力：局部坐标系口径 vs 基座坐标系口径
//   6) 静力学特例：q̇ = q̈ = 0 时 τ = ID(q,0,0) 就等于 G(q)，即"抵住重力所需力矩"
//   7) 用正动力学做仿真：KDL 自带的 RK4 积分器把状态前推
//
// 两个关键教学点：
//   a) ID 与 FD 互为逆运算，可以互相验证；三明治公式也可以和 RNE 结果互相验证，
//      这两条闭环是本示例的核心。
//   b) KDL 的 f_ext 要求外力表达在**各段自己的局部坐标系**里，
//      这与"沿世界 Z 轴往下压"的直觉写法不同，必须做坐标变换。

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_dynamics_print.hpp"
#include "kdl_dynparam.hpp"
#include "kdl_fdynamics.hpp"
#include "kdl_fk.hpp"
#include "kdl_idynamics.hpp"
#include "kdl_kinematics_print.hpp"
#include "kdl_tools.hpp"

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
 * @return max|a(i) − b(i)|；长度取较小者。
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
 * @brief 用给定数值填充一个 JntArray（多余的自由度保持为 0）。
 * @param n      数组长度。
 * @param values 数值数组。
 * @param count  values 的长度。
 * @return 填充好的 JntArray。
 */
KDL::JntArray makeJntArray(unsigned int n, const double * values, unsigned int count)
{
  KDL::JntArray array(n);
  for (unsigned int i = 0; i < n && i < count; ++i) {
    array(i) = values[i];
  }
  return array;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamics_demo");

  // ---- 0. 建链：链上的每个段都必须带惯量，否则动力学结果全是 0 ----
  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    RCLCPP_ERROR(node->get_logger(), "建链失败，退出");
    rclcpp::shutdown();
    return 1;
  }
  const unsigned int n = chain.getNrOfJoints();
  std::cout << "\nchain: " << n << " joints, " << chain.getNrOfSegments() << " segments\n";

  // ---- 1. 输入：一组关节角 / 速度 / 加速度 ----
  const double q_values[] = {0.3, -0.4, 0.6, 0.2, -0.3, 0.5};      // rad
  const double qdot_values[] = {0.1, -0.2, 0.3, 0.0, 0.15, -0.25};  // rad/s
  const double qddot_values[] = {0.5, 0.4, -0.3, 0.2, -0.1, 0.1};   // rad/s²

  const KDL::JntArray q = makeJntArray(n, q_values, 6);
  const KDL::JntArray qdot = makeJntArray(n, qdot_values, 6);
  const KDL::JntArray qddot = makeJntArray(n, qddot_values, 6);

  std::cout << "\n>>> 1) 输入\n";
  std::cout << "  q     (rad)    ";  kdl_kinematics::printJntArray(q);
  std::cout << "  qdot  (rad/s)  ";  kdl_kinematics::printJntArray(qdot);
  std::cout << "  qddot (rad/s²) ";  kdl_kinematics::printJntArray(qddot);

  // ---- 2. 逆动力学：τ = ID(q, q̇, q̈) ----
  std::cout << "\n>>> 2) 逆动力学 τ = ID(q, qdot, qddot)\n";
  const kdl_dynamics::IdResult id_result =
    kdl_dynamics::inverseDynamics(chain, q, qdot, qddot);
  kdl_dynamics::printIdResult(id_result);

  // ---- 3. 三项分解：τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q) ----
  std::cout << "\n>>> 3) 三项分解与对照\n";

  const kdl_dynamics::MassResult mass_result = kdl_dynamics::jointSpaceInertia(chain, q);
  kdl_dynamics::printMassResult(mass_result);
  std::cout << "  M 对称?      " << (kdl_dynamics::isSymmetric(mass_result.mass) ? "是" : "否") << "\n";
  std::cout << "  M 正定?      "
            << (kdl_dynamics::isPositiveDefinite(mass_result.mass) ? "是" : "否")
            << "   (物理上必须如此)\n";

  const kdl_dynamics::CoriolisResult coriolis_result =
    kdl_dynamics::coriolisTorque(chain, q, qdot);
  kdl_dynamics::printCoriolisResult(coriolis_result);

  const kdl_dynamics::GravityResult gravity_result = kdl_dynamics::gravityTorque(chain, q);
  kdl_dynamics::printGravityResult(gravity_result);

  // 把三项按公式拼起来：τ = M·q̈ + C·q̇ + G
  KDL::JntArray m_times_qddot(n);
  KDL::Multiply(mass_result.mass, qddot, m_times_qddot);

  KDL::JntArray assembled(n);
  for (unsigned int i = 0; i < n; ++i) {
    assembled(i) = m_times_qddot(i) + coriolis_result.torque(i) + gravity_result.torque(i);
  }

  std::cout << "\n  拼接结果 M·q̈ + C·q̇ + G:\n";
  kdl_kinematics::printJntArray(assembled);
  std::cout << "  与 RNE 结果的最大偏差(应≈0): "
            << maxAbsDiff(assembled, id_result.torque) << " N·m\n";
  std::cout << "  => 两条完全不同的算法路径给出同一个答案，公式得到验证。\n";

  // ---- 4. 正动力学，以及 FD(ID(q̈)) 往返闭合 ----
  std::cout << "\n>>> 4) 正动力学 qddot = FD(q, qdot, τ)\n";
  const kdl_dynamics::FdResult fd_result =
    kdl_dynamics::forwardDynamics(chain, q, qdot, id_result.torque);
  kdl_dynamics::printFdResult(fd_result);
  std::cout << "  与原始 qddot 的最大偏差(应≈0): " << maxAbsDiff(fd_result.qddot, qddot)
            << " rad/s²\n";
  std::cout << "  => ID 与 FD 互为逆运算，往返后回到出发点。\n";

  // ---- 5. 末端施加外力：两种坐标系口径 ----
  std::cout << "\n>>> 5) 末端施加外力（同样是 10 N 向下压，两种坐标系写法）\n";

  // 写法一：直接按基座坐标系给 —— "沿世界 Z 轴向下压 10 N"。
  const KDL::Wrench push_in_base(
    KDL::Vector(0.0, 0.0, -10.0), KDL::Vector::Zero());
  std::cout << "  基座坐标系下的外力:\n";
  kdl_dynamics::printWrench(push_in_base);

  // 写法二：换算成末端局部坐标系再给。KDL 原生要的就是这种口径。
  KDL::Frame tip_frame;
  if (!kdl_kinematics::forwardKinematics(chain, q, tip_frame)) {
    rclcpp::shutdown();
    return 1;
  }
  // tip_frame.M 是 R_base←tip，取逆即 R_tip←base：v_tip = M⁻¹ · v_base。
  const KDL::Rotation R_tip_from_base = tip_frame.M.Inverse();
  const KDL::Wrench push_in_tip(
    R_tip_from_base * push_in_base.force,
    R_tip_from_base * push_in_base.torque);
  std::cout << "  换算到末端局部坐标系后:\n";
  kdl_dynamics::printWrench(push_in_tip);

  const kdl_dynamics::IdResult id_wrench_base =
    kdl_dynamics::inverseDynamicsWithTipWrenchBase(chain, q, qdot, qddot, push_in_base);
  const kdl_dynamics::IdResult id_wrench_tip =
    kdl_dynamics::inverseDynamicsWithTipWrenchLocal(chain, q, qdot, qddot, push_in_tip);

  std::cout << "\n  两种口径解出的关节力矩:\n";
  kdl_dynamics::printIdResult(id_wrench_base);
  kdl_dynamics::printIdResult(id_wrench_tip);
  std::cout << "  两者最大偏差(应≈0): "
            << maxAbsDiff(id_wrench_base.torque, id_wrench_tip.torque) << " N·m\n";

  KDL::JntArray torque_delta(n);
  for (unsigned int i = 0; i < n; ++i) {
    torque_delta(i) = id_wrench_base.torque(i) - id_result.torque(i);
  }
  std::cout << "  相比无外力时的变化量:\n";
  kdl_kinematics::printJntArray(torque_delta);

  // ---- 6. 静力学特例：静止时 τ = G(q) ----
  std::cout << "\n>>> 6) 静力学特例：qdot = qddot = 0 时 τ 应当只剩重力项\n";
  const KDL::JntArray qdot_zero(n);
  const KDL::JntArray qddot_zero(n);
  const kdl_dynamics::IdResult static_id =
    kdl_dynamics::inverseDynamics(chain, q, qdot_zero, qddot_zero);
  kdl_dynamics::printIdResult(static_id);
  std::cout << "  与 G(q) 的最大偏差(应≈0): "
            << maxAbsDiff(static_id.torque, gravity_result.torque) << " N·m\n";
  std::cout << "  => 机器人静止时仍需出力抵住重力；这个 τ = G(q) 本身就是「重力补偿力矩」，"
               "直接施加即可（不要取负，取负会把重力加倍）。\n";

  // ---- 7. 用正动力学做仿真：RK4 积分 ----
  std::cout << "\n>>> 7) 正动力学用于仿真：先做重力补偿，再给定力矩，用 RK4 前推 0.2 s\n";
  std::cout << "  (KDL 的 RK4Integrator 签名较长，这里直接调用以展示原始 API)\n";

  KDL::ChainFdSolver_RNE fd_solver(chain, kdl_dynamics::defaultGravity());
  KDL::Wrenches f_ext_zero(chain.getNrOfSegments(), KDL::Wrench::Zero());

  // 从第 1 步那个位形静止出发（用零位形也可以，只是零位形下整条臂几乎
  // 竖在 joint1 的轴线上，绕 joint1 的等效惯量极小，一动就飞快）。
  KDL::JntArray q_sim = q;
  KDL::JntArray qdot_sim(n);
  KDL::JntArray qddot_sim(n);
  KDL::JntArray dq(n);
  KDL::JntArray dq_dot(n);
  KDL::JntArray q_temp(n);
  KDL::JntArray qdot_temp(n);

  // 恒定力矩 = 初始位形的重力补偿 G(q0) + 在 joint1 上多给 2 N·m 让它动起来。
  // 注意：gravityTorque 返回的 G(q) 就是"抵住重力所需力矩"，**直接施加，不要取负**
  //       （取负会把重力加倍，臂会加速砸下去）。
  // 只补偿一次 G(q0) 是有意为之：机器人动起来后重力项会变，
  // 真实控制器需要每个周期重算，这里正好能从仿真结果里看出这点偏差。
  const kdl_dynamics::GravityResult g0 = kdl_dynamics::gravityTorque(chain, q_sim);
  KDL::JntArray torque_cmd(n);
  for (unsigned int i = 0; i < n; ++i) {
    torque_cmd(i) = g0.torque(i);
  }
  torque_cmd(0) += 2.0;  // joint1 额外 2 N·m

  std::cout << "  力矩指令 (N·m):\n";
  kdl_kinematics::printJntArray(torque_cmd);

  unsigned int nj = n;
  // 注意 KDL 把这些参数声明成了 const double& / double&，
  // 其中 t 是 const 引用，所以并不会像它文档里写的那样被更新。
  double t = 0.0;
  double dt = 0.001;
  const unsigned int total_steps = 200;  // 200 * 1 ms = 0.2 s
  const unsigned int print_every = 50;

  std::cout << "\n  步数    t (s)    q[0] (rad)   qdot[0] (rad/s)   qddot[0] (rad/s²)\n";
  std::cout << "  " << 0 << "       0         " << q_sim(0) << "        " << qdot_sim(0)
            << "          " << qddot_sim(0) << "\n";

  for (unsigned int step = 1; step <= total_steps; ++step) {
    fd_solver.RK4Integrator(
      nj, t, dt, q_sim, qdot_sim, torque_cmd, f_ext_zero, fd_solver,
      qddot_sim, dq, dq_dot, q_temp, qdot_temp);

    if (step % print_every == 0) {
      std::cout << "  " << step << "       " << (step * dt) << "      " << q_sim(0)
                << "        " << qdot_sim(0) << "          " << qddot_sim(0) << "\n";
    }
  }
  std::cout << "  => 恒定力矩下 joint1 平稳加速，这就是正动力学配合积分器的仿真循环。\n";
  std::cout << "     注意重力补偿只在初始位形算过一次，机器人动起来后重力项会变化，\n"
            << "     真实控制器需要每个周期重算 G(q)，否则残余重力会带来持续偏差。\n";

  rclcpp::shutdown();
  return 0;
}
