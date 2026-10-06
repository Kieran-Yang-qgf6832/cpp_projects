// Copyright (c) 2026, kdl_tools authors.
// 教学用途：把 KDL 的数据结构"讲人话"地打印到终端。
//
// 单独放在一个文件里，是为了和 kdl_tools.hpp（只负责构建）解耦：
// 构建逻辑不需要知道"怎么打印"，打印逻辑也不关心"怎么构建"。

#ifndef KDL_TOOLS__KDL_TOOLS_PRINT_HPP_
#define KDL_TOOLS__KDL_TOOLS_PRINT_HPP_

#include <iostream>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

namespace kdl_tools
{

// ---------------------------------------------------------------------------
// 叶子级：单个 joint / segment
// ---------------------------------------------------------------------------

/**
 * @brief 单行打印一个关节的关键信息（名字、类型、轴）。
 * @param joint 待打印的 KDL 关节。
 * @param os    输出流，默认 std::cout；也支持 std::ostringstream。
 *
 * @note KDL::Joint 只保存"关节怎么动"（类型 + 轴），
 *       不保存上下限、阻尼等；那些信息在 urdf::Model 里。
 */
void printJoint(const KDL::Joint & joint, std::ostream & os = std::cout);

/**
 * @brief 展开打印一个段的完整信息（名字、父关节、到末端变换、惯量）。
 * @param segment 待打印的段（一个 link 对应一个 Segment）。
 * @param os      输出流，默认 std::cout。
 *
 * @note Segment = Joint + "从关节到本段末端的固定变换"(FrameToTip) + 惯量。
 */
void printSegment(const KDL::Segment & segment, std::ostream & os = std::cout);

// ---------------------------------------------------------------------------
// 容器级：整条链 / 整棵树
// ---------------------------------------------------------------------------

/**
 * @brief 逐段打印一条 KDL::Chain，并在末尾汇总关节类型统计。
 * @param chain 待打印的链。
 * @param os    输出流，默认 std::cout。
 *
 * @note 链里可能含有 fixed（固定）段，它们不是可动关节，
 *       所以"段数(segments)"通常大于"自由度数(joints)"。
 */
void printChain(const KDL::Chain & chain, std::ostream & os = std::cout);

/**
 * @brief 递归打印整棵 KDL::Tree 的层级结构。
 * @param tree 待打印的树。
 * @param os   输出流，默认 std::cout。
 *
 * @note 输出形如（当前 4 轴模型，根 link 就是 base_link，没有 world link）：
 *         [root] base_link
 *           └─ joint1 (RotAxis(绕轴转))  -> link1
 *             └─ joint2 (RotAxis(绕轴转))  -> link2
 *               ...
 *       缩进表示树中层级，括号里是关节类型。
 */
void printTree(const KDL::Tree & tree, std::ostream & os = std::cout);

}  // namespace kdl_tools

#endif  // KDL_TOOLS__KDL_TOOLS_PRINT_HPP_
