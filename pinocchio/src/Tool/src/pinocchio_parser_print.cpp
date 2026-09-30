/**
 * @file pinocchio_parser_print.cpp
 * @brief pinocchio_parser_print.hpp 中打印函数的实现。
 */

#include "pinocchio_parser_print.hpp"

#include <cstddef>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>

namespace pinocchio_tools
{

namespace
{

/// 打印单个关节的限位信息时使用的列宽。
constexpr int kColumnWidth = 12;

/**
 * @brief 打印模型的无效状态提示，供各打印函数复用。
 *
 * @param[in] robot 待检查的模型对象。
 * @param[out] os   输出流。
 * @return 模型无效返回 true（此时已输出提示）；有效返回 false。
 */
bool printInvalid(const RobotModel & robot, std::ostream & os)
{
  if (robot.valid)
  {
    return false;
  }
  os << "[invalid model] " << robot.message << std::endl;
  return true;
}

/**
 * @brief 把一个 Eigen 列向量格式化成定长字符串，便于表格列对齐。
 *
 * @details Eigen 的 operator<< 会忽略 std::setw，因此先用 ostringstream 生成字符串，
 *          再由调用方套用 std::setw 输出。
 *
 * @tparam Derived Eigen 表达式类型。
 * @param[in] vector 待格式化的列向量。
 * @return 形如 "[0.100 -0.200]" 的字符串。
 */
template<typename Derived>
std::string formatVector(const Eigen::MatrixBase<Derived> & vector)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << vector.transpose();
  return stream.str();
}

}  // namespace

std::string frameTypeToString(pinocchio::FrameType type)
{
  switch (type)
  {
  case pinocchio::OP_FRAME:
    return "OP_FRAME";
  case pinocchio::JOINT:
    return "JOINT";
  case pinocchio::FIXED_JOINT:
    return "FIXED_JOINT";
  case pinocchio::BODY:
    return "BODY";
  case pinocchio::SENSOR:
    return "SENSOR";
  default:
    return "UNKNOWN";
  }
}

void printModelSummary(const RobotModel & robot, std::ostream & os)
{
  os << "==================== model summary ====================" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  os << "  name         : " << robot.name << std::endl;
  os << "  source       : " << robot.source_path << std::endl;
  os << "  base         : " << (robot.fixed_base ? "fixed" : "free-flyer") << std::endl;
  os << "  nq / nv      : " << robot.model.nq << " / " << robot.model.nv << std::endl;
  os << "  njoints      : " << robot.model.njoints << " (universe included)" << std::endl;
  os << "  nbodies      : " << robot.model.nbodies << std::endl;
  os << "  nframes      : " << robot.model.nframes << std::endl;
  os << "  joints       : " << robot.numJointsExcludingUniverse() << " (universe excluded)"
     << std::endl;
  os << "======================================================" << std::endl;
}

void printJointList(const RobotModel & robot, std::ostream & os)
{
  os << "---------------------------- joints ----------------------------" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  const pinocchio::Model & model = robot.model;

  // id 0 是 universe 伪关节，它不贡献任何自由度，其类型字段只是占位实现，故不在此表中展示。
  os << "note: id 0 (universe) is an internal pseudo-joint and is omitted." << std::endl;
  os << std::left << std::setw(5) << "id" << std::setw(14) << "name" << std::setw(30) << "type"
     << std::right << std::setw(4) << "nq" << std::setw(4) << "nv" << std::setw(7) << "idx_q"
     << std::setw(7) << "idx_v" << std::setw(8) << "parent" << std::endl;

  for (int i = 1; i < model.njoints; ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    const auto & joint = model.joints[index];

    os << std::left << std::setw(5) << i << std::setw(14) << model.names[index] << std::setw(30)
       << pinocchio::shortname(joint) << std::right << std::setw(4) << pinocchio::nq(joint)
       << std::setw(4) << pinocchio::nv(joint) << std::setw(7) << pinocchio::idx_q(joint)
       << std::setw(7) << pinocchio::idx_v(joint) << std::setw(8) << model.parents[index]
       << std::endl;
  }
  os << "----------------------------------------------------------------" << std::endl;
}

void printFrameList(const RobotModel & robot, std::ostream & os)
{
  os << "---------------------------- frames ----------------------------" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  const pinocchio::Model & model = robot.model;

  os << std::left << std::setw(5) << "id" << std::setw(16) << "name" << std::setw(15) << "type"
     << std::right << std::setw(9) << "parentJ" << std::setw(9) << "parentF" << std::endl;

  for (int i = 0; i < model.nframes; ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    const auto & frame = model.frames[index];

    os << std::left << std::setw(5) << i << std::setw(16) << frame.name << std::setw(15)
       << frameTypeToString(frame.type) << std::right << std::setw(9) << frame.parentJoint
       << std::setw(9) << frame.parentFrame << std::endl;
  }
  os << "----------------------------------------------------------------" << std::endl;
}

void printLimitList(const RobotModel & robot, std::ostream & os)
{
  os << "---------------------------- limits ----------------------------" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  const pinocchio::Model & model = robot.model;

  os << std::left << std::setw(5) << "id" << std::setw(12) << "name" << std::right
     << std::setw(kColumnWidth) << "lower" << std::setw(kColumnWidth) << "upper"
     << std::setw(kColumnWidth) << "velocity" << std::setw(kColumnWidth) << "effort" << std::endl;

  for (int i = 1; i < model.njoints; ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    const auto & joint = model.joints[index];

    const int nq = pinocchio::nq(joint);
    const int nv = pinocchio::nv(joint);
    const int idx_q = pinocchio::idx_q(joint);
    const int idx_v = pinocchio::idx_v(joint);

    os << std::left << std::setw(5) << i << std::setw(12) << model.names[index] << std::right;

    // 位置限位定义在配置向量上，速度与力矩限位定义在切空间向量上。
    // 这里对每个关节取对应的子向量打印，从而同时兼容单自由度与多自由度关节。
    os << std::setw(kColumnWidth) << formatVector(model.lowerPositionLimit.segment(idx_q, nq))
       << std::setw(kColumnWidth) << formatVector(model.upperPositionLimit.segment(idx_q, nq))
       << std::setw(kColumnWidth) << formatVector(model.upperVelocityLimit.segment(idx_v, nv))
       << std::setw(kColumnWidth) << formatVector(model.upperEffortLimit.segment(idx_v, nv))
       << std::endl;
  }
  os << "----------------------------------------------------------------" << std::endl;
}

void printInertiaList(const RobotModel & robot, std::ostream & os)
{
  os << "--------------------------- inertias ---------------------------" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  const pinocchio::Model & model = robot.model;

  for (int i = 0; i < model.njoints; ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    const pinocchio::Inertia & inertia = model.inertias[index];

    // id 0 是 universe 伪关节：URDF 中经固定关节挂在根部的连杆，其惯性会被合并到此处。
    os << "  [" << i << "] " << model.names[index];
    if (i == 0)
    {
      os << " (root bodies merged into the universe)";
    }
    os << ": mass = " << inertia.mass() << " kg, com = " << inertia.lever().transpose() << " m"
       << std::endl;
    os << inertia.inertia().matrix().format(
      Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "      [", "]", "",
                      ""));
    os << std::endl;
  }
  os << "----------------------------------------------------------------" << std::endl;
}

void printFramePlacement(const RobotModel & robot, const std::string & frame_name, std::ostream & os)
{
  os << "---------------------- frame placement -------------------------" << std::endl;
  if (printInvalid(robot, os))
  {
    return;
  }

  const pinocchio::Model & model = robot.model;
  if (!model.existFrame(frame_name))
  {
    os << "[frame not found] " << frame_name << std::endl;
    return;
  }

  const pinocchio::FrameIndex frame_id = model.getFrameId(frame_name);
  const pinocchio::SE3 & placement = robot.data.oMf[frame_id];

  os << "  frame       : " << frame_name << " (id = " << frame_id << ")" << std::endl;
  os << "  translation : " << placement.translation().transpose() << " m" << std::endl;
  os << "  rotation    :" << std::endl;
  os << placement.rotation().format(
    Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "      [", "]", "",
                    ""));
  os << std::endl;
  os << "----------------------------------------------------------------" << std::endl;
}

}  // namespace pinocchio_tools
