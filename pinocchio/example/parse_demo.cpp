/**
 * @file parse_demo.cpp
 * @brief 示例一：解析 URDF 并打印模型的全部元信息。
 *
 * @details 运行方式（需要先 source 安装目录）：
 *          @code
 *          ros2 run pinocchio_tools parse_demo                       # 使用包内默认模型，固定基座
 *          ros2 run pinocchio_tools parse_demo --free-flyer           # 以浮动基座方式解析
 *          ros2 run pinocchio_tools parse_demo /path/to/robot.urdf    # 解析指定 URDF
 *          @endcode
 *
 *          该示例演示了 Tool 模块的最小用法：
 *          parseUrdf() 得到 RobotModel，再用打印工具输出关节、帧、限位与惯性信息。
 */

#include "pinocchio_parser.hpp"
#include "pinocchio_parser_print.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace
{

/// 本项目自带的默认模型在安装目录下的相对路径。
constexpr const char * kDefaultModelName = "robotic_arm.urdf";

/// 命令行中用于切换浮动基座的开关。
constexpr const char * kFreeFlyerFlag = "--free-flyer";

/**
 * @brief 解析后的命令行参数。
 */
struct CommandLine
{
  /// 待解析的 URDF 路径。
  std::string urdf_path;

  /// 是否以浮动基座方式解析。
  bool free_flyer = false;

  /// 是否请求打印帮助。
  bool help = false;
};

/**
 * @brief 解析命令行参数，未提供路径时回退到包内的默认模型。
 *
 * @details 支持的形式为 parse_demo [--free-flyer] [urdf_path]，参数顺序任意。
 *
 * @param[in] argc main 的参数个数。
 * @param[in] argv main 的参数数组。
 * @return 解析得到的命令行参数；路径无法确定时 urdf_path 为空字符串。
 */
CommandLine parseCommandLine(int argc, char ** argv)
{
  CommandLine cli;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);
    if (arg == kFreeFlyerFlag)
    {
      cli.free_flyer = true;
    }
    else if (arg == "-h" || arg == "--help")
    {
      cli.help = true;
    }
    else
    {
      cli.urdf_path = arg;
    }
  }

  if (cli.help || !cli.urdf_path.empty())
  {
    return cli;
  }

  try
  {
    const std::string share_dir = ament_index_cpp::get_package_share_directory("pinocchio_tools");
    cli.urdf_path = share_dir + "/model/" + kDefaultModelName;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Failed to locate the package share directory: " << e.what() << std::endl;
  }

  return cli;
}

}  // namespace

int main(int argc, char ** argv)
{
  const CommandLine cli = parseCommandLine(argc, argv);
  if (cli.help)
  {
    std::cout << "Usage: parse_demo [--free-flyer] [urdf_path]" << std::endl;
    return 0;
  }
  if (cli.urdf_path.empty())
  {
    std::cerr << "Usage: parse_demo [--free-flyer] [urdf_path]" << std::endl;
    return 1;
  }

  std::cout << "Parsing URDF: " << cli.urdf_path << std::endl << std::endl;

  pinocchio_tools::ParseOptions options;
  options.root_joint_type =
    cli.free_flyer ? pinocchio_tools::RootJointType::kFreeFlyer : pinocchio_tools::RootJointType::kFixed;

  const pinocchio_tools::RobotModel robot = pinocchio_tools::parseUrdf(cli.urdf_path, options);
  if (!robot.valid)
  {
    std::cerr << "Parse failed: " << robot.message << std::endl;
    return 1;
  }

  pinocchio_tools::printModelSummary(robot);
  std::cout << std::endl;
  pinocchio_tools::printJointList(robot);
  std::cout << std::endl;
  pinocchio_tools::printFrameList(robot);
  std::cout << std::endl;
  pinocchio_tools::printLimitList(robot);
  std::cout << std::endl;
  pinocchio_tools::printInertiaList(robot);

  return 0;
}
