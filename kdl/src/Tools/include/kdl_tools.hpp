// Copyright (c) 2026, kdl_tools authors.
// 教学用途：本文件只做一件事——把 URDF 变成 KDL 的数据结构。
//
// KDL 本体（orocos_kdl）只管运动学/动力学计算，本身并不认识 URDF 这种
// XML 机器人描述格式。真正干活的是 kdl_parser：
//
//     URDF 文本 ──urdf::parseURDF*──► urdf::ModelInterface ──kdl_parser──► KDL::Tree
//
// 拿到 KDL::Tree 之后，再用 Tree::getChain(base, tip) 截取其中一条链，
// 就得到了做正/逆运动学最常用的 KDL::Chain。

#ifndef KDL_TOOLS__KDL_TOOLS_HPP_
#define KDL_TOOLS__KDL_TOOLS_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
// urdf/model.h 里是完整的 urdf::Model（含 initFile/initString），
// 它继承自 urdf_model/model.h 中的 urdf::ModelInterface。
#include <urdf/model.h>

namespace kdl_tools
{

/// 本机器人（src/model/robotic_arm.urdf）整棵树所对应的默认链两端。
/// 之所以用常量而不是写死在函数里，是为了让"默认值"这一约定有唯一出处。
constexpr const char * kDefaultBaseLink = "base_link";
constexpr const char * kDefaultTipLink = "link4";

// ---------------------------------------------------------------------------
// 一、URDF 文本 -> KDL::Tree
// ---------------------------------------------------------------------------

/**
 * @brief 从 URDF 文件路径构建 KDL::Tree（最常用入口）。
 * @param urdf_file URDF 文件的绝对路径或相对路径，例如
 *                  "/opt/ros/.../arm/model/robotic_arm.urdf"。
 * @param tree      [输出] 构建好的运动学树；成功时被填充，失败时保持原样。
 * @return true 表示解析成功；false 表示文件打不开或 URDF 内容非法。
 *
 * @note 这是教学用的"便利封装"，内部顺序为：
 *       1) urdf::parseURDFFile(file) 得到 urdf::ModelInterfaceSharedPtr
 *       2) kdl_parser::treeFromUrdfModel(model, tree) 转换为 KDL::Tree
 *       失败原因会通过 RCLCPP_ERROR 打印出来，便于课堂排查。
 */
bool buildTreeFromUrdfFile(const std::string & urdf_file, KDL::Tree & tree);

/**
 * @brief 从内存中的 URDF 字符串构建 KDL::Tree。
 * @param urdf_xml URDF 的完整 XML 文本（例如从 ROS 参数 "robot_description" 读到）。
 * @param tree     [输出] 构建好的运动学树。
 * @return true 表示解析成功。
 *
 * @note 在 ROS 中，节点常把 URDF 放在参数 robot_description 里，
 *       此时用本重载比先落盘成文件更方便。
 */
bool buildTreeFromUrdfString(const std::string & urdf_xml, KDL::Tree & tree);

/**
 * @brief 从已解析好的 urdf::ModelInterface 构建 KDL::Tree。
 * @param robot_model 已经解析完成的 URDF 模型（kdl_parser 的底层入参）。
 * @param tree        [输出] 构建好的运动学树。
 * @return true 表示转换成功。
 *
 * @note 这是最底层的重载：调用方若已经持有 urdf::Model，
 *       就无需重复解析 XML，可直接复用。
 */
bool buildTreeFromUrdfModel(
  const urdf::ModelInterface & robot_model, KDL::Tree & tree);

// ---------------------------------------------------------------------------
// 二、Tree -> KDL::Chain
// ---------------------------------------------------------------------------

/**
 * @brief 在整棵树里截取从 base_link 到 tip_link 的一条运动学链。
 * @param tree    已构建好的运动学树。
 * @param chain   [输出] 截取出来的链；成功时被填充，失败时保持原样。
 * @param base_link 链的起点（基座）link 名，默认 kDefaultBaseLink（"base_link"）。
 * @param tip_link  链的终点（末端）link 名，默认 kDefaultTipLink（"link4"）。
 * @return true 表示两个 link 之间存在一条连通路径。
 *
 * @note 若省略后两个参数，就是本机器人最常用的 "base_link -> link4"。
 *       本模型的根 link 就是 base_link（URDF 里没有 world link），
 *       所以 base_link 必须传 "base_link"。
 * @note KDL::Tree::getChain() 的语义是：
 *       - 两者都在树中且连通 -> 成功；
 *       - 任一 link 不存在 / 不连通 -> 返回 false。
 */
bool buildChain(
  const KDL::Tree & tree, KDL::Chain & chain,
  const std::string & base_link = kDefaultBaseLink,
  const std::string & tip_link = kDefaultTipLink);

/**
 * @brief 直接由 URDF 文件一步得到 KDL::Chain（buildTreeFromUrdfFile + buildChain）。
 * @param urdf_file URDF 文件路径。
 * @param chain     [输出] 截取出来的链。
 * @param base_link 链的起点 link 名，默认 "base_link"。
 * @param tip_link  链的终点 link 名，默认 "link4"。
 * @return true 表示从文件到链的整条流程全部成功。
 *
 * @note 只关心一条链时用这个最省事；需要遍历整棵树时请用 buildTreeFromUrdfFile。
 */
bool buildChainFromUrdfFile(
  const std::string & urdf_file, KDL::Chain & chain,
  const std::string & base_link = kDefaultBaseLink,
  const std::string & tip_link = kDefaultTipLink);

// ---------------------------------------------------------------------------
// 三、辅助：加载 urdf::Model（备用超能力）
// ---------------------------------------------------------------------------

/**
 * @brief 解析 URDF 文件为 urdf::Model，方便额外读取质量、惯量、关节限位。
 * @param urdf_file URDF 文件路径。
 * @param model     [输出] 解析得到的模型。
 * @return true 表示解析成功。
 *
 * @note KDL 只搬运运动学信息（关节类型/轴/位姿），像 link 的 mass、
 *       inertia、joint 的 limit 这些，KDL::Segment/Joint 里并没有完整保留，
 *       需要查这些物理量时就要回到 urdf::Model 来读。
 */
bool loadUrdfModel(const std::string & urdf_file, urdf::Model & model);

}  // namespace kdl_tools

#endif  // KDL_TOOLS__KDL_TOOLS_HPP_
