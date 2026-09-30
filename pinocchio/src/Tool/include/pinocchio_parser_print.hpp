/**
 * @file pinocchio_parser_print.hpp
 * @brief pinocchio_parser 的打印工具：把模型内容以可读形式输出到流。
 *
 * @details 该文件与 pinocchio_parser.hpp 分离，遵循「解析与展示职责单一」的原则：
 *          - pinocchio_parser 只负责得到数据；
 *          - 本文件只负责把数据打印出来，不修改任何模型内容。
 *          所有函数都接受一个 std::ostream，默认 std::cout，便于重定向到日志或文件。
 */

#pragma once

#include "pinocchio_parser.hpp"

#include <iostream>
#include <ostream>
#include <string>

namespace pinocchio_tools
{

/**
 * @brief 把 Pinocchio 的帧类型枚举转换为可读字符串。
 *
 * @param[in] type 帧类型（见 pinocchio::FrameType）。
 * @return 类型名称，如 "JOINT"、"FIXED_JOINT"、"BODY"；未知类型返回 "UNKNOWN"。
 */
std::string frameTypeToString(pinocchio::FrameType type);

/**
 * @brief 打印模型总览：名称、来源、基座类型、各维数与数量统计。
 *
 * @param[in] robot 待打印的模型对象（允许是解析失败的对象，此时只打印错误信息）。
 * @param[out] os   输出流，默认标准输出。
 */
void printModelSummary(const RobotModel & robot, std::ostream & os = std::cout);

/**
 * @brief 打印关节列表：索引、名称、关节类型、nq/nv 及其在配置/速度向量中的起始下标、父关节。
 *
 * @param[in] robot 待打印的模型对象。
 * @param[out] os   输出流，默认标准输出。
 */
void printJointList(const RobotModel & robot, std::ostream & os = std::cout);

/**
 * @brief 打印帧列表：索引、名称、帧类型、所属关节与父帧索引。
 *
 * @param[in] robot 待打印的模型对象。
 * @param[out] os   输出流，默认标准输出。
 */
void printFrameList(const RobotModel & robot, std::ostream & os = std::cout);

/**
 * @brief 打印关节限位：位置上下限、速度上限与力矩上限。
 *
 * @param[in] robot 待打印的模型对象。
 * @param[out] os   输出流，默认标准输出。
 */
void printLimitList(const RobotModel & robot, std::ostream & os = std::cout);

/**
 * @brief 打印各连杆惯性参数：质量、质心位置（在关节坐标系下）与 3x3 转动惯量。
 *
 * @param[in] robot 待打印的模型对象。
 * @param[out] os   输出流，默认标准输出。
 */
void printInertiaList(const RobotModel & robot, std::ostream & os = std::cout);

/**
 * @brief 打印指定帧在当前位置下的齐次变换（平移与旋转）。
 *
 * @details 调用前需先执行过正向运动学（如
 *          pinocchio::forwardKinematics + pinocchio::updateFramePlacements），
 *          否则数据缓冲区中的位姿不是最新的。
 *
 * @param[in] robot      待打印的模型对象，其 data.oMf 需已更新。
 * @param[in] frame_name 目标帧名称，例如 "link6"。
 * @param[out] os        输出流，默认标准输出。
 */
void printFramePlacement(
  const RobotModel & robot, const std::string & frame_name, std::ostream & os = std::cout);

}  // namespace pinocchio_tools
