// Copyright (c) 2026, kdl_identification authors.
// 教学用途：kdl_regressor.hpp 中声明的函数在这里落地。
//
// 文件顺序与头文件一一对应：
//   一、参数向量的布局        —— 下标与名字的映射
//   二、参数 <-> 链 的转换    —— 本模块唯一"需要点数学"的地方（平行轴定理）
//   三、回归矩阵              —— 精确差分：每列 = 把一个参数加一后的 τ 增量
//
// 为什么差分是"精确"的：逆动力学对惯性参数严格线性，即
//     ID(β + Δ) − ID(β) = Jacobian(β) · Δ
// 与 β 无关。所以取 Δ = e_k（只有第 k 个参数 +1），差商就直接是该列，
// 没有任何截断误差。取纯数值微分的通用公式反而会引入误差，这里的做法
// 是"借线性性质把精确求值当微分用"。

#include "kdl_regressor.hpp"

#include <cmath>
#include <string>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_identification
{
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_identification";

/// 判定"质量为零"的阈值（kg）。
constexpr double kMassEpsilon = 1e-12;

/// 判定"第一阶矩为零"的阈值（kg·m）。
constexpr double kFirstMomentEpsilon = 1e-12;

/**
 * @brief 把 KDL::RotationalInertia 取成标准的 3×3 惯量矩阵。
 * @param inertia [in] KDL 的转动惯量。
 * @return 矩阵 [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]。
 *
 * @note 不去读内部 data[9]（不同版本的内存布局可能变），而是用
 *       RotationalInertia::operator*(Vector) —— 它算的是角动量 I·ω，
 *       取 ω = e_x/e_y/e_z 就正好得到三列，接口稳定且可读。
 */
Eigen::Matrix3d rotationalInertiaToMatrix(const KDL::RotationalInertia & inertia)
{
  const KDL::Vector col0 = inertia * KDL::Vector(1.0, 0.0, 0.0);
  const KDL::Vector col1 = inertia * KDL::Vector(0.0, 1.0, 0.0);
  const KDL::Vector col2 = inertia * KDL::Vector(0.0, 0.0, 1.0);

  Eigen::Matrix3d matrix;
  matrix << col0.x(), col1.x(), col2.x(), col0.y(), col1.y(), col2.y(), col0.z(), col1.z(),
    col2.z();
  return matrix;
}

/**
 * @brief 由标准惯性参数造一个 KDL::RigidBodyInertia。
 * @param params   [in] 10 维参数块：m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz。
 * @param segment  [in] 段名（仅用于报错）。
 * @param inertia  [out] 构造出的刚体惯量。
 * @param message  [out] 失败原因。
 * @return true 表示成功。
 *
 * @note β 里的转动惯量是**相对参考点**的 I_ref，而 KDL 构造函数要的是
 *       **相对 COG** 的 I_cog，换算用平行轴定理：
 *           I_ref = I_cog + m·(|c|²·Id − c·cᵀ)
 *           ⇒ I_cog = I_ref − m·(|c|²·Id − c·cᵀ),  c = h / m
 */
bool makeRigidBodyInertia(
  const Eigen::VectorXd & params, const std::string & segment, KDL::RigidBodyInertia & inertia,
  std::string & message)
{
  const double m = params[0];
  const Eigen::Vector3d h(params[1], params[2], params[3]);

  Eigen::Matrix3d i_ref;
  i_ref << params[4], params[5], params[6], params[5], params[7], params[8], params[6], params[8],
    params[9];

  if (m > kMassEpsilon) {
    const Eigen::Vector3d c = h / m;
    const Eigen::Matrix3d i_cog =
      i_ref - m * (c.dot(c) * Eigen::Matrix3d::Identity() - c * c.transpose());
    const KDL::RotationalInertia rot(
      i_cog(0, 0), i_cog(1, 1), i_cog(2, 2), i_cog(0, 1), i_cog(0, 2), i_cog(1, 2));
    inertia = KDL::RigidBodyInertia(m, KDL::Vector(c.x(), c.y(), c.z()), rot);
    return true;
  }

  // m ≈ 0 的退化情形：KDL 构造函数里 h 恒为 m·c = 0，所以只有第一阶矩也为 0
  // 时才能表示；这时 COG 取参考点，I_ref 直接就是 I_cog。
  if (h.norm() > kFirstMomentEpsilon) {
    message = "段 " + segment + " 的质量接近 0 但第一阶矩不为 0，无法用 KDL 表示";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", message.c_str());
    return false;
  }
  const KDL::RotationalInertia rot(
    i_ref(0, 0), i_ref(1, 1), i_ref(2, 2), i_ref(0, 1), i_ref(0, 2), i_ref(1, 2));
  inertia = KDL::RigidBodyInertia(0.0, KDL::Vector::Zero(), rot);
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、参数向量的布局
// ---------------------------------------------------------------------------

const char * parameterName(unsigned int index)
{
  static const char * kNames[kParamsPerLink] = {
    "m", "m*cx", "m*cy", "m*cz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};
  return (index < kParamsPerLink) ? kNames[index] : "?";
}

unsigned int parameterCount(const KDL::Chain & chain)
{
  return kParamsPerLink * chain.getNrOfSegments();
}

std::string parameterLabel(const KDL::Chain & chain, unsigned int index)
{
  const unsigned int segment = index / kParamsPerLink;
  if (segment >= chain.getNrOfSegments()) {
    return "<invalid>";
  }
  return chain.getSegment(segment).getName() + "." + parameterName(index % kParamsPerLink);
}

// ---------------------------------------------------------------------------
// 二、参数 <-> 链 的转换
// ---------------------------------------------------------------------------

bool extractParameters(
  const KDL::Chain & chain, Eigen::VectorXd & parameters, std::string & message)
{
  const unsigned int segments = chain.getNrOfSegments();
  parameters = Eigen::VectorXd::Zero(kParamsPerLink * segments);

  for (unsigned int s = 0; s < segments; ++s) {
    const KDL::RigidBodyInertia & inertia = chain.getSegment(s).getInertia();
    const double m = inertia.getMass();

    // KDL 存的是 m·c（第一阶矩）；m ≈ 0 时 getCOG() 直接返回 0，第一阶矩
    // 不可还原，按 0 处理（真实连杆不会有这个问题）。
    const KDL::Vector cog = inertia.getCOG();
    const Eigen::Vector3d h = (m > kMassEpsilon) ?
      Eigen::Vector3d(m * cog.x(), m * cog.y(), m * cog.z()) : Eigen::Vector3d::Zero();
    const Eigen::Matrix3d i_ref = rotationalInertiaToMatrix(inertia.getRotationalInertia());

    auto block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);
    block[0] = m;
    block[1] = h.x();
    block[2] = h.y();
    block[3] = h.z();
    block[4] = i_ref(0, 0);
    block[5] = i_ref(0, 1);
    block[6] = i_ref(0, 2);
    block[7] = i_ref(1, 1);
    block[8] = i_ref(1, 2);
    block[9] = i_ref(2, 2);
  }

  message.clear();
  return true;
}

bool applyParameters(
  const KDL::Chain & chain, const Eigen::VectorXd & parameters, KDL::Chain & result,
  std::string & message)
{
  const unsigned int segments = chain.getNrOfSegments();
  if (parameters.size() != static_cast<Eigen::Index>(kParamsPerLink * segments)) {
    message = "参数向量长度必须等于 10 × 段数（" + std::to_string(kParamsPerLink * segments) + "）";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", message.c_str());
    return false;
  }

  // KDL::Chain 的拷贝是深拷贝，改副本的段惯量不会影响模板链。
  KDL::Chain rebuilt = chain;

  for (unsigned int s = 0; s < segments; ++s) {
    const Eigen::VectorXd block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);
    KDL::RigidBodyInertia inertia;
    if (!makeRigidBodyInertia(block, chain.getSegment(s).getName(), inertia, message)) {
      return false;
    }
    rebuilt.getSegment(s).setInertia(inertia);
  }

  result = rebuilt;
  message.clear();
  return true;
}

// ---------------------------------------------------------------------------
// 三、回归矩阵
// ---------------------------------------------------------------------------

RegressorBuilder::RegressorBuilder(const KDL::Chain & chain, const KDL::Vector & gravity)
: gravity_(gravity), joints_(chain.getNrOfJoints()), segments_(chain.getNrOfSegments())
{
  const unsigned int params = kParamsPerLink * segments_;
  if (joints_ == 0 || params == 0) {
    error_ = "链为空（没有关节或没有段），无法构建回归矩阵";
    return;
  }

  // 取名义参数 β₀，并据此重建"基准链"：所有差分列都相对它做。
  Eigen::VectorXd nominal_params;
  if (!extractParameters(chain, nominal_params, error_)) {
    return;
  }
  if (!applyParameters(chain, nominal_params, nominal_chain_, error_)) {
    return;
  }

  // 预先把"第 k 个参数 +1"的链全部造好。它们与采样点无关，所以只造一次。
  basis_chains_.resize(params);
  for (unsigned int k = 0; k < params; ++k) {
    Eigen::VectorXd perturbed = nominal_params;
    perturbed[k] += 1.0;
    if (!applyParameters(chain, perturbed, basis_chains_[k], error_)) {
      basis_chains_.clear();
      return;
    }
  }

  // 每个采样点都要跑 (params + 1) 次逆动力学。KDL 的求解器在构造时分配内部缓冲，
  // 若每次调用都重新构造，几千次评估下开销非常可观 —— 所以这里把求解器也缓存起来，
  // 每次采样只做一次 CartToJnt。注意求解器内部持有对链的引用：上面 basis_chains_
  // 已经 resize 定容、之后不再改动，元素地址稳定，引用是安全的。
  solvers_.reserve(params + 1);
  solvers_.push_back(std::make_unique<KDL::ChainIdSolver_RNE>(nominal_chain_, gravity_));
  for (unsigned int k = 0; k < params; ++k) {
    solvers_.push_back(std::make_unique<KDL::ChainIdSolver_RNE>(basis_chains_[k], gravity_));
  }

  external_wrenches_.assign(segments_, KDL::Wrench::Zero());
  nominal_torque_ = KDL::JntArray(joints_);
  column_torque_ = KDL::JntArray(joints_);

  valid_ = true;
}

bool RegressorBuilder::build(
  const KDL::JntArray & q, const KDL::JntArray & qdot, const KDL::JntArray & qddot,
  Eigen::MatrixXd & Y, std::string & message) const
{
  if (!valid_) {
    message = error_.empty() ? "回归矩阵构造器不可用" : error_;
    return false;
  }
  if (q.rows() != joints_ || qdot.rows() != joints_ || qddot.rows() != joints_) {
    message = "q/qdot/qddot 的长度必须都等于 " + std::to_string(joints_);
    return false;
  }

  const unsigned int params = parameterCount();
  Y = Eigen::MatrixXd::Zero(joints_, params);

  // Y(:, k) = ID(β₀ + e_k) − ID(β₀)
  const int base_code =
    solvers_[0]->CartToJnt(q, qdot, qddot, external_wrenches_, nominal_torque_);
  if (base_code < 0) {
    message = std::string("逆动力学求解失败：") + solvers_[0]->strError(base_code);
    return false;
  }

  for (unsigned int k = 0; k < params; ++k) {
    const int code =
      solvers_[k + 1]->CartToJnt(q, qdot, qddot, external_wrenches_, column_torque_);
    if (code < 0) {
      message = "第 " + std::to_string(k) + " 列的逆动力学求解失败：" +
        solvers_[k + 1]->strError(code);
      return false;
    }
    for (unsigned int j = 0; j < joints_; ++j) {
      Y(j, k) = column_torque_(j) - nominal_torque_(j);
    }
  }

  message.clear();
  return true;
}

RegressorResult RegressorBuilder::build(const std::vector<RegressorSample> & samples) const
{
  RegressorResult result;
  result.joints = joints_;
  result.segments = segments_;
  result.samples = static_cast<unsigned int>(samples.size());

  if (!valid_) {
    result.message = error_.empty() ? "回归矩阵构造器不可用" : error_;
    return result;
  }
  if (samples.empty()) {
    result.message = "采样点为空，无法构建回归矩阵";
    return result;
  }

  const unsigned int params = parameterCount();
  result.regressor = Eigen::MatrixXd::Zero(joints_ * result.samples, params);
  for (unsigned int i = 0; i < result.samples; ++i) {
    const RegressorSample & s = samples[i];
    Eigen::MatrixXd sample_regressor;
    if (!build(s.q, s.qdot, s.qddot, sample_regressor, result.message)) {
      result.message = "第 " + std::to_string(i) + " 个采样点：" + result.message;
      return result;
    }
    result.regressor.block(i * joints_, 0, joints_, params) = sample_regressor;
  }

  result.success = true;
  result.message.clear();
  return result;
}

RegressorResult buildRegressor(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Vector & gravity)
{
  RegressorBuilder builder(chain, gravity);
  RegressorSample sample;
  sample.q = q;
  sample.qdot = qdot;
  sample.qddot = qddot;
  RegressorResult result = builder.build(std::vector<RegressorSample>{sample});
  if (!result.success) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
  }
  return result;
}

RegressorResult buildRegressor(
  const KDL::Chain & chain, const std::vector<RegressorSample> & samples,
  const KDL::Vector & gravity)
{
  RegressorBuilder builder(chain, gravity);
  RegressorResult result = builder.build(samples);
  if (!result.success) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
  }
  return result;
}

}  // namespace kdl_identification
