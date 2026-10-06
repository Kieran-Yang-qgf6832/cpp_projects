// Copyright (c) 2026, kdl_tools authors.
// 教学示例：把 src/Tools 里的工具串起来跑一遍。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools build_from_urdf
//   ros2 run kdl_tools build_from_urdf --tip_link link4
//
// 本示例演示三件事：
//   1) URDF 文件  -> KDL::Tree          （buildTreeFromUrdfFile）
//   2) KDL::Tree  -> KDL::Chain         （buildChain，首末 link 可配）
//   3) 打印 & 逐段访问 Segment/Joint     （kdl_tools_print + KDL 原生接口）

#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "kdl_tools.hpp"
#include "kdl_tools_print.hpp"

namespace
{

/// 由 CMake 在编译期传入（见 CMakeLists.txt 的 KDL_TOOLS_MODEL_DIR）。
/// 这样示例既能从源码树直接运行，也不依赖运行时的包安装路径查询。
#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

/**
 * @brief 定位示例用到的 URDF 文件。
 * @param node 用于读取可选的 urdf_file 参数。
 * @return URDF 的路径。
 *
 * @note 优先级：ROS 参数 urdf_file > 源码树 src/model/robotic_arm.urdf。
 */
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

/**
 * @brief 打印命令行用法说明。
 */
void printUsage()
{
  std::cout
    << "usage: build_from_urdf [--base_link <name>] [--tip_link <name>]\n"
    << "  default: base_link -> link4\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ---- 0. 解析简单的命令行参数（--base_link / --tip_link）----
  std::string base_link = kdl_tools::kDefaultBaseLink;
  std::string tip_link = kdl_tools::kDefaultTipLink;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--base_link" && i + 1 < argc) {
      base_link = argv[++i];
    } else if (arg == "--tip_link" && i + 1 < argc) {
      tip_link = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      printUsage();
      rclcpp::shutdown();
      return 0;
    }
  }

  auto node = std::make_shared<rclcpp::Node>(
    "build_from_urdf_demo",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  const std::string urdf_file = resolveUrdfPath(*node);  // NOLINT(readability/nonconst)
  std::cout << "URDF file: " << urdf_file << "\n\n";

  // ---- 1. URDF 文件 -> KDL::Tree ----
  KDL::Tree tree("robot");
  if (!kdl_tools::buildTreeFromUrdfFile(urdf_file, tree)) {
    RCLCPP_ERROR(node->get_logger(), "failed to build KDL::Tree, exit.");
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "\n>>> 1) 从 URDF 构建出的运动学树\n";
  kdl_tools::printTree(tree);

  // ---- 2. KDL::Tree -> KDL::Chain ----
  KDL::Chain chain;
  if (!kdl_tools::buildChain(tree, chain, base_link, tip_link)) {
    RCLCPP_ERROR(
      node->get_logger(), "failed to build chain %s -> %s, exit.",
      base_link.c_str(), tip_link.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "\n>>> 2) 截取出的运动学链 (" << base_link << " -> " << tip_link << ")\n";
  kdl_tools::printChain(chain);

  // ---- 3. 逐个访问链中的 Segment / Joint，演示原生 KDL 接口 ----
  std::cout << "\n>>> 3) 逐段访问（chain.getSegment(i) 拿到 Segment，再取 Joint）\n";
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Segment & segment = chain.getSegment(i);
    std::cout << "segment[" << i << "] = " << segment.getName() << "\n";
    kdl_tools::printJoint(segment.getJoint());
  }

  // ---- 4. 需要质量/限位时，回到 urdf::Model ----
  // KDL 只搬运运动学信息；质量、惯量、关节上下限要去 urdf::Model 读。
  urdf::Model model;
  if (kdl_tools::loadUrdfModel(urdf_file, model)) {
    std::cout << "\n>>> 4) 从 urdf::Model 读取第一个 link 的物理属性\n";
    const auto link_it = model.links_.find(base_link);
    if (link_it != model.links_.end() && link_it->second->inertial) {
      const urdf::Inertial & inertial = *link_it->second->inertial;
      std::cout << "link '" << base_link << "' mass = " << inertial.mass << " kg, "
                << "com = [" << inertial.origin.position.x << ", "
                << inertial.origin.position.y << ", "
                << inertial.origin.position.z << "]\n";
    } else {
      std::cout << "link '" << base_link << "' has no <inertial> block\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}
