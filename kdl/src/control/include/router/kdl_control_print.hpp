// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务层的打印工具。
//
// 与其它模块同一个立场：**求解逻辑不关心打印**。control 的所有算法都不调用这
// 里的函数，只有示例/上层调试时才用。所以改这里的输出格式不会动到解算结果。
//
// 打印内容复用底层模块的打印函数（轨迹概览、路点表、限位表），避免同一种信息
// 在两个模块里各写一遍、格式还不一致。

#ifndef KDL_CONTROL__KDL_CONTROL_PRINT_HPP_
#define KDL_CONTROL__KDL_CONTROL_PRINT_HPP_

#include <iostream>

#include "router/kdl_task_base.hpp"

namespace kdl_control
{

/**
 * @brief 打印机器人上下文：链的规模、关节行程、力矩上限与已启用的约束。
 * @param ctx [in] 机器人上下文。
 * @param os  [in,out] 输出流，默认 std::cout。
 */
void printRobotContext(const RobotContext & ctx, std::ostream & os = std::cout);

/**
 * @brief 打印任务状态。
 * @param status [in] 任务状态。
 * @param os     [in,out] 输出流。
 */
void printTaskStatus(TaskStatus status, std::ostream & os = std::cout);

/**
 * @brief 打印一次解算结果：结论、失败原因、轨迹概览与质量指标。
 * @param result [in] 解算结果。
 * @param os     [in,out] 输出流。
 *
 * @note 路点表只在段数较少时打印（笛卡尔任务重建出来的轨迹可能上千段，全打出来
 *       只会把终端刷满）；段数多时只打概览与结论，要看细节请自己调
 *       kdl_interpolation::printKnotTable()。
 */
void printControlResult(const ControlResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印力矩前馈序列的峰值与几个采样点。
 * @param result [in] 含 torque_feedforward 的解算结果。
 * @param os     [in,out] 输出流。
 */
void printTorqueFeedforward(const ControlResult & result, std::ostream & os = std::cout);

}  // namespace kdl_control

#endif  // KDL_CONTROL__KDL_CONTROL_PRINT_HPP_
