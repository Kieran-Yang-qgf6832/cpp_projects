// Copyright (c) 2026, kdl_kinematics authors.
// 4 轴臂（平面 2R + 2 自由度腕部）的解析逆解。设计与推导见 kdl_ik_analytic.hpp。

#include "kdl_ik_analytic.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <kdl/segment.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_fk.hpp"

namespace kdl_kinematics
{
namespace
{

constexpr const char * kLogTag = "kdl_ik_analytic";

/// 结构判据容差：单位向量点积的"平行/垂直"判定，以及"长度为零"的判定。
/// 取 1e-4（≈0.006°）而不是更严：URDF 由 SolidWorks 导出，rpy 只保留 4~5 位小数，
/// 轴向本身就带 ~1e-5 的噪声；再严会把正常的模型判成结构不符。
constexpr double kStructureTol = 1e-4;

/// 牛顿迭代收敛判据（残差模长）。
constexpr double kNewtonTol = 1e-13;

/// 判定"目标不可达"的残差阈值。
constexpr double kReachTol = 1e-6;

/// 有限差分求 2x2 雅可比的步长。
constexpr double kFiniteDiffStep = 1e-7;

/// 把向量 X 相对 O 分解成平面内坐标 (·u, ·v) 与平面外坐标 (·axis)。
struct PlaneCoords
{
  double pu = 0.0;
  double pv = 0.0;
  double pn = 0.0;
};

PlaneCoords toPlane(
  const KDL::Vector & X, const KDL::Vector & O,
  const KDL::Vector & u, const KDL::Vector & v, const KDL::Vector & axis)
{
  const KDL::Vector d = X - O;
  return PlaneCoords{KDL::dot(d, u), KDL::dot(d, v), KDL::dot(d, axis)};
}

/**
 * @brief 在 q=0 处沿链逐段复合，取各关节的原点与轴方向。
 *
 * @note KDL 约定 Segment::pose(q) = Joint::pose(q) * f_tip，关节位于该段的参考原点，
 *       所以第 i 段的参考原点就是第 i 个关节的位置；轴方向由该段的 JointAxis()
 *       经累积旋转搬到基座系。
 */
void collectJointFrames(
  const KDL::Chain & chain, std::vector<KDL::Vector> & O,
  std::vector<KDL::Vector> & A, KDL::Frame & tip)
{
  const unsigned int n = chain.getNrOfJoints();
  O.assign(n, KDL::Vector::Zero());
  A.assign(n, KDL::Vector::Zero());

  KDL::Frame T;
  for (unsigned int i = 0; i < n; ++i) {
    const KDL::Segment & seg = chain.getSegment(i);
    // KDL 的 f_tip 是"从关节末端到段末端"的变换，所以关节原点在 f_tip 之后：
    // O[i] 取 T*f_tip 的位置，而轴方向用**变换前**的 T.M 搬运（实测这样才与
    // URDF 里的关节原点/轴一致）。把这两者放反会让整组几何量错位一格。
    O[i] = (T * seg.getFrameToTip()).p;
    const KDL::Vector a = T.M * seg.getJoint().JointAxis();
    A[i] = a / a.Norm();
    T = T * seg.pose(0.0);
  }
  tip = T;
}

/// 由 (q1,q2) 正算 joint3 原点 O3 与 joint3 轴方向 a3（基座系）。
/// 这是牛顿迭代里反复调用的"小 FK"，只算到 joint3 为止。
void forwardToJoint3(
  const Planar4ArmGeometry & geo, double q1, double q2, KDL::Vector & O3, KDL::Vector & a3)
{
  const double theta1 = q1 + geo.delta1;
  const double theta2 = theta1 - q2 + geo.delta2;

  const double xu = geo.L1 * std::cos(theta1) + geo.L2 * std::cos(theta2);
  const double xv = geo.L1 * std::sin(theta1) + geo.L2 * std::sin(theta2);

  O3 = geo.origin + xu * geo.u + xv * geo.v + geo.out_of_plane * geo.axis;

  const double phi = theta2 - geo.delta2;  // joint3 轴的平面方位角
  a3 = std::cos(phi) * geo.u + std::sin(phi) * geo.v;
}

/// 牛顿迭代的残差：r0 = (p−O3)·a3 − h，r1 = |p−O3| − |c|。见头文件第四节。
void residual(
  const Planar4ArmGeometry & geo, const KDL::Vector & p, double q1, double q2,
  double tool_radius, double r[2])
{
  KDL::Vector O3, a3;
  forwardToJoint3(geo, q1, q2, O3, a3);
  const KDL::Vector y = p - O3;
  r[0] = KDL::dot(y, a3) - geo.wrist_h;
  r[1] = y.Norm() - tool_radius;
}

AnalyticIkResult makeError(const KDL::JntArray & seed, int code, const std::string & msg)
{
  AnalyticIkResult result;
  result.q = seed;
  result.error_code = code;
  result.message = msg;
  RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", msg.c_str());
  return result;
}

}  // namespace

bool extractPlanar4ArmGeometry(
  const KDL::Chain & chain, Planar4ArmGeometry & geo, std::string & message)
{
  geo = Planar4ArmGeometry{};
  message.clear();

  if (chain.getNrOfJoints() != kPlanar4ArmJoints) {
    message = "解析解只支持 4 关节链，当前链有 " + std::to_string(chain.getNrOfJoints()) +
      " 个关节";
    return false;
  }
  if (chain.getNrOfSegments() != kPlanar4ArmJoints) {
    message = "链里有固定段（段数 " + std::to_string(chain.getNrOfSegments()) +
      " != 关节数 4），平面 2R 的推导不成立";
    return false;
  }

  std::vector<KDL::Vector> O, A;
  KDL::Frame tip;
  collectJointFrames(chain, O, A, tip);

  // ---- 结构判据（任一不满足就如实失败，不硬着头皮给错几何）----
  if (std::abs(std::abs(KDL::dot(A[0], A[1])) - 1.0) > kStructureTol) {
    message = "joint1 与 joint2 的轴不平行（点积 " + std::to_string(KDL::dot(A[0], A[1])) +
      "），不构成平面 2R";
    return false;
  }
  if (std::abs(KDL::dot(A[0], A[2])) > kStructureTol) {
    message = "joint3 的轴不垂直于 joint1 的轴（点积 " + std::to_string(KDL::dot(A[0], A[2])) +
      "），joint3 轴不落在臂平面内";
    return false;
  }
  if (std::abs(KDL::dot(A[2], A[3])) > kStructureTol) {
    message = "joint3 与 joint4 的轴不正交（点积 " + std::to_string(KDL::dot(A[2], A[3])) +
      "），腕部不是两级正交回转";
    return false;
  }

  geo.axis = A[0];
  geo.origin = O[0];

  // 臂平面内的正交基：u 取"q=0 时第一段连杆的指向"，v = axis × u（保证 u × v = axis）。
  const KDL::Vector d1 = O[1] - O[0];
  KDL::Vector u = d1 - KDL::dot(d1, geo.axis) * geo.axis;
  const double u_norm = u.Norm();
  if (u_norm < kStructureTol) {
    message = "joint1 与 joint2 重合，第一段连杆长度为零";
    return false;
  }
  geo.u = u / u_norm;
  geo.v = geo.axis * geo.u;

  // ---- 平面内连杆长度与平面外偏移 ----
  const PlaneCoords c1 = toPlane(O[1], O[0], geo.u, geo.v, geo.axis);
  const PlaneCoords c2 = toPlane(O[2], O[1], geo.u, geo.v, geo.axis);
  const PlaneCoords c3 = toPlane(O[2], O[0], geo.u, geo.v, geo.axis);

  geo.L1 = std::hypot(c1.pu, c1.pv);
  geo.L2 = std::hypot(c2.pu, c2.pv);
  geo.out_of_plane = c3.pn;

  if (geo.L1 < kStructureTol || geo.L2 < kStructureTol) {
    message = "平面内连杆长度为零（L1 = " + std::to_string(geo.L1) + "，L2 = " +
      std::to_string(geo.L2) + "）";
    return false;
  }

  // ---- 角度偏置：θ1 = q1 + delta1，θ2_abs = θ1 − q2 + delta2 ----
  geo.delta1 = std::atan2(c1.pv, c1.pu);
  geo.delta2 = std::atan2(c2.pv, c2.pu) - geo.delta1;

  // ---- 腕部：把 O3→工具点 按 joint3 轴分解 ----
  const KDL::Vector c = tip.p - O[2];
  geo.wrist_h = KDL::dot(c, A[2]);
  const KDL::Vector c_perp = c - geo.wrist_h * A[2];
  geo.tool_perp_radius = c_perp.Norm();

  if (geo.tool_perp_radius < kStructureTol) {
    message = "工具点落在 joint3 轴上（|c⊥| ≈ 0），q3 无法由位置确定";
    return false;
  }

  // 方位角偏置：记录 q=0 处 c⊥ 在基座系里的方向。求 q3 时先把它按 q1−q2 绕 axis 转
  // 到当前构型（link3 在 q3=0 时的朝向就是 Rot(axis, q1−q2) 作用在 q=0 朝向上），
  // 再解"绕 a3 转多少能把 c⊥ 转到目标方向"。
  geo.tool_perp_ref = c_perp;

  geo.valid = true;
  message.clear();
  return true;
}

// ---------------------------------------------------------------------------
// 三、绕工具轴滚转：把一个 6 维参考姿态投影成 4 轴臂能实现的那一维
// ---------------------------------------------------------------------------

bool makeRollFrame(const KDL::Chain & chain, const KDL::JntArray & q, RollFrame & out)
{
  out = RollFrame{};

  const unsigned int n = chain.getNrOfJoints();
  if (n < 4 || q.rows() != n) {
    return false;
  }

  // 工具轴怎么量出来：把第 4 个关节单独转 1 rad，两次 FK 的姿态之差就是"绕工具轴
  // 转 +1 rad"这个旋转，它的转轴按定义就是工具轴。见头文件第六节之后的说明。
  KDL::JntArray probe = q;
  probe(3) = 0.0;
  KDL::Frame f0;
  if (!forwardKinematics(chain, probe, f0)) {
    return false;
  }
  probe(3) = 1.0;
  KDL::Frame f1;
  if (!forwardKinematics(chain, probe, f1)) {
    return false;
  }

  KDL::Vector axis;
  const double angle = (f1.M * f0.M.Inverse()).GetRotAngle(axis, 1e-8);
  if (angle < 1e-3) {
    return false;  // 第 4 个关节转不动，滚转无从定义
  }
  out.axis = axis;

  // 参考向量：取末端坐标系的一个基向量，投到垂直于 axis 的平面上。三个基向量里
  // 总有至少一个不平行于 axis（axis 不可能同时平行于三个正交方向），取第一个够长的。
  const KDL::Vector basis[3] = {
    KDL::Vector(1.0, 0.0, 0.0), KDL::Vector(0.0, 1.0, 0.0), KDL::Vector(0.0, 0.0, 1.0)};
  for (int i = 0; i < 3; ++i) {
    KDL::Vector w = f0.M * basis[i];
    w = w - KDL::dot(w, out.axis) * out.axis;
    if (w.Norm() > 0.5) {
      out.w = w / w.Norm();
      out.local = basis[i];
      out.valid = true;
      return true;
    }
  }
  return false;
}

double rollAngle(const KDL::Rotation & rotation, const RollFrame & frame)
{
  KDL::Vector w = rotation * frame.local;
  w = w - KDL::dot(w, frame.axis) * frame.axis;
  const double norm = w.Norm();
  if (norm < 1e-9) {
    return 0.0;  // 退化：参考方向恰好平行于工具轴，滚转不可分辨
  }
  w = w / norm;
  return std::atan2(KDL::dot(frame.w * w, frame.axis), KDL::dot(frame.w, w));
}

AnalyticIkResult solveIkAnalytic(
  const KDL::Chain & chain, const Planar4ArmGeometry & geo,
  const KDL::Vector & target_position, double target_roll,
  const KDL::JntArray & q_seed, unsigned int max_iter)
{
  if (!geo.valid) {
    return makeError(q_seed, KDL::SolverI::E_NOT_UP_TO_DATE,
             "几何参数无效，请先成功调用 extractPlanar4ArmGeometry()");
  }
  if (chain.getNrOfJoints() != kPlanar4ArmJoints || q_seed.rows() != kPlanar4ArmJoints) {
    return makeError(q_seed, KDL::SolverI::E_SIZE_MISMATCH,
             "解析解要求 4 关节，且 q_seed 长度为 4");
  }

  const double tool_radius = std::hypot(geo.wrist_h, geo.tool_perp_radius);

  // ---- 牛顿法解 (q1,q2)：约束 (p−O3)·a3 = h 与 |p−O3| = |c| ----
  double q1 = q_seed(0);
  double q2 = q_seed(1);
  unsigned int iter = 0;
  double r[2] = {0.0, 0.0};
  for (; iter < max_iter; ++iter) {
    residual(geo, target_position, q1, q2, tool_radius, r);
    if (std::hypot(r[0], r[1]) < kNewtonTol) {
      break;
    }
    // 有限差分雅可比。不求解析导数：几何量全部来自实测的链，解析式要串一长串
    // 链式法则、容易写错，而 2x2 差分只多两次小 FK，代价可以忽略。
    double J[2][2];
    for (unsigned int col = 0; col < 2; ++col) {
      const double a1 = (col == 0) ? q1 + kFiniteDiffStep : q1;
      const double a2 = (col == 1) ? q2 + kFiniteDiffStep : q2;
      double rp[2];
      residual(geo, target_position, a1, a2, tool_radius, rp);
      J[0][col] = (rp[0] - r[0]) / kFiniteDiffStep;
      J[1][col] = (rp[1] - r[1]) / kFiniteDiffStep;
    }
    const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (std::abs(det) < 1e-14) {
      return makeError(q_seed, KDL::SolverI::E_NO_CONVERGE,
               "牛顿迭代的雅可比奇异（构型退化），无法继续");
    }
    q1 -= (J[1][1] * r[0] - J[0][1] * r[1]) / det;
    q2 -= (-J[1][0] * r[0] + J[0][0] * r[1]) / det;
  }

  residual(geo, target_position, q1, q2, tool_radius, r);
  const double reach_err = std::hypot(r[0], r[1]);
  if (reach_err >= kReachTol) {
    return makeError(q_seed, KDL::SolverI::E_OUT_OF_RANGE,
             "目标点超出工作空间（残差 " + std::to_string(reach_err) + "，迭代 " +
             std::to_string(iter) + " 步未收敛）");
  }

  // ---- 闭式解 q3：绕 a3 把 c⊥ 转到目标方向 ----
  KDL::Vector O3, a3;
  forwardToJoint3(geo, q1, q2, O3, a3);

  // c⊥ 在当前构型（q3=0）下的世界方向：把 q=0 的参考方向绕 axis 转过 q1−q2。
  const double plane_turn = q1 - q2;
  const KDL::Vector c_perp_now = KDL::Rotation::Rot(geo.axis, plane_turn) * geo.tool_perp_ref;

  // 目标方向：p = O3 + h·a3 + Rot(a3,q3)·c⊥  ⇒  Rot(a3,q3)·c⊥ = p − O3 − h·a3
  const KDL::Vector wanted = target_position - O3 - geo.wrist_h * a3;

  // 绕 a3 的转角：c⊥ 与 wanted 同在垂直于 a3 的平面内、模长相等，取带符号夹角即可。
  const double x = KDL::dot(c_perp_now, wanted);
  const double y = KDL::dot(c_perp_now * wanted, a3);
  const double q3 = std::atan2(y, x);

  AnalyticIkResult result;
  result.q = q_seed;
  result.q(0) = q1;
  result.q(1) = q2;
  result.q(2) = q3;
  result.q(3) = target_roll;  // 绕工具轴自转不影响工具点位置，直接采用给定值
  result.iterations = iter;
  return result;
}

}  // namespace kdl_kinematics
