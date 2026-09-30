/**
 * @file pinocchio_parser.cpp
 * @brief pinocchio_parser.hpp 中解析逻辑的实现。
 */

#include "pinocchio_parser.hpp"

#include <cstddef>
#include <exception>
#include <fstream>
#include <string>

namespace pinocchio_tools
{

namespace
{

/**
 * @brief 判断给定路径是否指向一个可读的文件。
 *
 * @param[in] path 待检查的文件路径。
 * @return 文件存在且可读返回 true，否则返回 false。
 */
bool fileIsReadable(const std::string & path)
{
  std::ifstream stream(path.c_str());
  return stream.good();
}

/**
 * @brief 从已构建完成的 Model 中汇总 RobotModel 的元信息。
 *
 * @details 该函数假定 @p robot.model 已经解析成功，会填充
 *          name / joint_names / frame_names 三个字段。
 *
 * @param[in,out] robot 目标模型对象，其元信息字段会被覆盖。
 */
void collectMetadata(RobotModel & robot)
{
  robot.name = robot.model.name;

  // 索引 0 固定为 universe 伪关节，不属于机器人本体，因此从 1 开始收集。
  robot.joint_names.clear();
  robot.joint_names.reserve(static_cast<std::size_t>(robot.model.njoints));
  for (int i = 1; i < robot.model.njoints; ++i)
  {
    robot.joint_names.push_back(robot.model.names[static_cast<std::size_t>(i)]);
  }

  robot.frame_names.clear();
  robot.frame_names.reserve(static_cast<std::size_t>(robot.model.nframes));
  for (int i = 0; i < robot.model.nframes; ++i)
  {
    robot.frame_names.push_back(robot.model.frames[static_cast<std::size_t>(i)].name);
  }
}

}  // namespace

RobotModel parseUrdf(const std::string & urdf_path, const ParseOptions & options)
{
  RobotModel robot;
  robot.source_path = urdf_path;
  robot.fixed_base = (options.root_joint_type == RootJointType::kFixed);

  if (urdf_path.empty())
  {
    robot.message = "URDF path is empty.";
    return robot;
  }

  if (!fileIsReadable(urdf_path))
  {
    robot.message = "URDF file is missing or not readable: " + urdf_path;
    return robot;
  }

  try
  {
    if (options.root_joint_type == RootJointType::kFreeFlyer)
    {
      pinocchio::urdf::buildModel(
        urdf_path, pinocchio::JointModelFreeFlyer(), options.root_joint_name, robot.model,
        options.verbose, options.mimic);
    }
    else
    {
      pinocchio::urdf::buildModel(urdf_path, robot.model, options.verbose, options.mimic);
    }
  }
  catch (const std::exception & e)
  {
    // 解析失败时复位模型，保证返回对象处于一致状态。
    robot.model = pinocchio::Model();
    robot.data = pinocchio::Data(robot.model);
    robot.message = std::string("Failed to parse URDF: ") + e.what();
    return robot;
  }

  // Data 必须依附于已经构建完成的 Model，二者维数要保持一致。
  robot.data = pinocchio::Data(robot.model);
  collectMetadata(robot);

  robot.valid = true;
  robot.message = "success";
  return robot;
}

}  // namespace pinocchio_tools
