/**
 * @file fk_demo.cpp
 * @brief 示例二：解析 URDF 后做一次正向运动学（FK）并打印各连杆位姿。
 *
 * @details 运行方式（需要先 source 安装目录）：
 *          @code
 *          ros2 run pinocchio_tools fk_demo                    # 使用包内默认模型
 *          ros2 run pinocchio_tools fk_demo /path/to/robot.urdf
 *          @endcode
 *
 *          该示例演示了解析结果如何直接用于算法：解析得到 RobotModel 之后，
 *          取出其 model / data 交给 pinocchio::forwardKinematics，
 *          再更新帧位姿并打印每个连杆的齐次变换。
 */

#include "pinocchio_parser.hpp"
#include "pinocchio_parser_print.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace
{

/// 本项目自带的默认模型在安装目录下的相对路径。
constexpr const char * kDefaultModelName = "robotic_arm.urdf";

/**
 * @brief 解析命令行参数，或在未提供参数时回退到包内的默认模型。
 *
 * @param[in] argc main 的参数个数。
 * @param[in] argv main 的参数数组。
 * @return 待解析的 URDF 路径；无法确定时返回空字符串。
 */
std::string resolveUrdfPath(int argc, char ** argv)
{
  if (argc > 1)
  {
    return std::string(argv[1]);
  }

  try
  {
    const std::string share_dir = ament_index_cpp::get_package_share_directory("pinocchio_tools");
    return share_dir + "/model/" + kDefaultModelName;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed to locate the package share directory: " << e.what() << std::endl;
    return std::string();
  }
}

/**
 * @brief 构造一个位于各关节限位内部的非零测试构型。
 *
 * @details 每个关节取其位置下限与上限之间 25% 处的值，保证构型非奇异且不越限。
 *
 * @param[in] robot 已解析成功的模型对象。
 * @return 长度为 robot.nq() 的广义位置向量。
 */
Eigen::VectorXd makeTestConfiguration(const pinocchio_tools::RobotModel & robot)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.model.nq);
  for (int i = 0; i < q.size(); ++i)
  {
    const double lower = robot.model.lowerPositionLimit[i];
    const double upper = robot.model.upperPositionLimit[i];
    q[i] = lower + 0.25 * (upper - lower);
  }
  return q;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string urdf_path = resolveUrdfPath(argc, argv);
  if (urdf_path.empty())
  {
    std::cerr << "Usage: fk_demo [urdf_path]" << std::endl;
    return 1;
  }

  pinocchio_tools::RobotModel robot = pinocchio_tools::parseUrdf(urdf_path);
  if (!robot.valid)
  {
    std::cerr << "Parse failed: " << robot.message << std::endl;
    return 1;
  }

  std::cout << "Model: " << robot.name << " (nq = " << robot.nq() << ", nv = " << robot.nv() << ")"
            << std::endl;

  const Eigen::VectorXd q = makeTestConfiguration(robot);
  std::cout << "q = " << q.transpose() << std::endl << std::endl;

  // 正向运动学：更新关节位姿；updateFramePlacements 进一步更新所有帧的位姿。
  pinocchio::forwardKinematics(robot.model, robot.data, q);
  pinocchio::updateFramePlacements(robot.model, robot.data);

  // 依次打印每个连杆帧在基座坐标系下的位姿。
  const std::vector<std::string> frame_names = {
    "link1", "link2", "link3", "link4", "link5", "link6"};
  for (const std::string & frame_name : frame_names)
  {
    pinocchio_tools::printFramePlacement(robot, frame_name);
  }

  return 0;
}
