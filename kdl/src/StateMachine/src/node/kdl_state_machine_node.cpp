// Copyright (c) 2026, kdl_tools authors.
// 教学用途：状态机节点的可执行入口（状态机本体与任务发送函数都在 include/kdl_state_machine.hpp）。
//
// 模块布局：include/ 只有 kdl_state_machine.hpp 一个头；src/ 放实现，本文件在 src/node/ 下
// （可执行入口与库实现分开，和 src/control/src/node/kdl_control_node.cpp 同一约定）。
//
// 与 kdl_control_node 的两个不同：
//   1) 本节点**不需要阻塞等任务执行完**：任务经 sendTask() 异步发出，状态机用定时器
//      轮询结果，所以不会出现"service 回调把执行器占住"的问题；
//   2) 但执行器仍取多线程：本节点没有私有线程池，多线程能让 service / 定时器 /
//      状态订阅在有别的节点占着线程时仍然被及时调度。
//
// 注意：状态机里所有回调都在默认的**互斥**回调组，多线程执行器不会让它们并发，
//       所以 state_ 等运行期成员不需要加锁（见头文件说明）。
//
// 本文件是最小入口，**不装任何任务来源**：只发一条任务就调
// node->startJointTask(...) / node->startCartesianTask(...)；多条任务的链条则调
// node->setTaskSource(...)。两者都没调时，状态机等依赖就绪后会直接进 finished。

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kdl_state_machine.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<kdl_state_machine::KdlStateMachine> node;
  try {
    node = std::make_shared<kdl_state_machine::KdlStateMachine>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("state_machine"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
