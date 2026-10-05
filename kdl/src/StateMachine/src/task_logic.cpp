// Copyright (c) 2026, kdl_tools authors.
// 教学用途：用状态机的**任务启动函数**串起一条多步作业流程（本文件就是"流程定义"）。
//
// 流程（两步都是关节空间；数组格式 = [q1, ..., q6, duration]，末位是时长 [s]）：
//   ① 关节空间 → [0.3, 0.6, 0.2, 0.4, 0.5, 0.0]   6 s
//   ② 关节空间 → [0.9, 0.5, 0.3, 0.2, 0.4, 0.0]   6 s
//
// 为什么不能"连着调两次 startJointTask()"：
//   状态机是**单飞行**的 —— 有一条在飞时 sendTask() 返回 kBusy，startTask() 也只在
//   kIdle / kFinished / kFailed 接受（否则返回 false + "状态机正在运行…"）。
//   所以必须等上一条进终态再发下一条。这里用状态机公开的 `~/state` 话题做完成信号，
//   订阅的是节点私有名 `~/state`，节点改名 / 加命名空间都不受影响。
//
// 运行（先起仿真 + 控制器 + kdl_control_node，见 README）：
//   ros2 run kdl_tools task_logic
//   ros2 topic echo /state_machine/state      # 另一终端观察
//
// 注意：本文件把 auto_start 显式关掉（auto_start=false）—— 流程完全由下面的 TaskFlow
//   驱动；若开着 auto_start，状态机会在依赖就绪后先自己跑一轮（此时没有任务来源，
//   会直接进 finished），与本文件的推进相互干扰。

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "kdl_state_machine.hpp"

namespace
{

using kdl_state_machine::KdlStateMachine;
using kdl_state_machine::TaskKind;

/// 流程的一步：用哪个启动函数（kind）+ 数值数组（末位是时长 [s]）。
struct Step
{
  TaskKind kind;
  std::vector<double> values;
};

/// 流程定义：改这里就是改作业流程。
const std::vector<Step> kSteps{
  {TaskKind::kJointSpace, {0.3, 0.6, 0.2, 0.4, 0.3, 0.0, 6.0}},
  {TaskKind::kJointSpace, {0.9, 0.5, 0.3, 0.2, 0.2, 0.0, 6.0}},
};

/**
 * @brief 把 `~/state` 的内容截成状态名。
 *
 * @note `~/state` 发的是 `stateName` 或 `failed：<原因>`：状态名是纯 ASCII，
 *       "：" 和原因都是非 ASCII 字节，所以"截到第一个非 ASCII 字节"即可。
 */
std::string stateNameOf(const std::string & message)
{
  std::size_t end = 0;
  while (end < message.size() && static_cast<unsigned char>(message[end]) < 0x80) {
    ++end;
  }
  return message.substr(0, end);
}

/**
 * @brief 按 kSteps 顺序驱动状态机：每见一次终态就启动下一步。
 *
 * @note 订阅 `~/state`（keep_last(1) + transient_local）：晚订阅也能立刻拿到当前状态，
 *       所以"第一步"也由这个订阅触发，不需要额外 kick。
 * @note 回调与状态机的定时器同属默认互斥回调组 → 彼此串行，在这里调 startTask()
 *       不会和 tick() 抢 state_（这也是头文件里推荐的调用时机）。
 */
class TaskFlow
{
public:
  TaskFlow(const std::shared_ptr<KdlStateMachine> & machine, std::vector<Step> steps)
  : machine_(machine), steps_(std::move(steps))
  {
    state_sub_ = machine_->create_subscription<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) { onState(msg->data); });
  }

private:
  void onState(const std::string & state)
  {
    const std::string name = stateNameOf(state);

    // 只有 kIdle / kFinished / kFailed 才轮得到我们推进；其余状态说明还在跑。
    if (name != "idle" && name != "finished" && name != "failed") {
      return;
    }
    if (done_) {
      return;
    }
    if (name == "failed") {
      done_ = true;
      RCLCPP_ERROR(machine_->get_logger(), "任务失败，流程中止：%s", state.c_str());
      return;
    }
    if (next_ >= steps_.size()) {
      done_ = true;
      RCLCPP_INFO(machine_->get_logger(), "任务流程全部完成（共 %zu 步）", steps_.size());
      return;
    }
    startStep(next_);
  }

  /// 启动第 index 步：按 kind 分派到对应的启动函数。
  void startStep(std::size_t index)
  {
    const Step & step = steps_[index];
    std::string message =
      "task_logic 不认识的任务类型（TaskKind = " +
      std::to_string(static_cast<int>(step.kind)) + "）";
    bool ok = false;

    switch (step.kind) {
      case TaskKind::kJointSpace:
        ok = machine_->startJointTask(step.values, message);
        break;
      case TaskKind::kCartesianSpace:
        ok = machine_->startCartesianTask(step.values, message);
        break;
        // ---- 预留：新增任务类型时在这里加分支 ----
    }

    if (!ok) {
      done_ = true;
      RCLCPP_ERROR(machine_->get_logger(), "第 %zu 步启动失败：%s", index + 1, message.c_str());
      return;
    }

    ++next_;
    RCLCPP_INFO(
      machine_->get_logger(), "已启动第 %zu/%zu 步", index + 1, steps_.size());
  }

  std::shared_ptr<KdlStateMachine> machine_;
  std::vector<Step> steps_;
  std::size_t next_ = 0;  ///< 待启动的步号
  bool done_ = false;     ///< 已全部完成或已中止
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // auto_start 显式关掉：流程由 TaskFlow 驱动（理由见文件头）。
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("auto_start", false)});

  std::shared_ptr<KdlStateMachine> node;
  try {
    node = std::make_shared<KdlStateMachine>(options);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("task_logic"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  auto flow = std::make_shared<TaskFlow>(node, kSteps);
  (void)flow;  // 由 executor 驱动；持有到 spin 结束即可

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
