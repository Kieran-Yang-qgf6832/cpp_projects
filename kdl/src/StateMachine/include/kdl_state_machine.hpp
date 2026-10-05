// Copyright (c) 2026, kdl_tools authors.
// 教学用途：任务级状态机 + 独立的任务发送函数（本模块**只有这一个头文件**）。
//
// ===========================================================================
// 这一层在整条链路里的位置
// ===========================================================================
//   [状态机] ──sendTask()──▶ srv /control_task ──▶ kdl_control_node
//   [任意调用者] ─┘                                  │ ① TaskRouter::dispatch() 解算
//                                                    │ ② 按 point_dt 离散 + 逆动力学
//          ◀── pollTask() 取结果 ── 响应              ▼
//                                            topic /control_reference
//                                                    ▼
//                                     kdl_effort_controller（500 Hz 内算力矩）
//                                                    ▼
//                                                MuJoCo <motor>
//
// ===========================================================================
// 结构：发送与管理分离（本头文件分三层看）
// ===========================================================================
//   一、任务的定义   TaskKind / TaskTarget / TaskResult
//                    —— 只描述"要做什么"和"结果如何"，**不含"现在走到哪一步"**。
//
//   二、任务发送     TaskHandle + **自由函数** sendTask / pollTask / discardTask /
//                    taskInFlight / serviceReady / buildRequest
//                    —— 目标由参数传入，函数**不依赖状态机的任何内部状态**；
//                       句柄（service client + 在飞的请求）由**调用者**持有。
//                       想发一条任务，不需要状态机（见"用法 A"）。
//
//   三、任务状态机   KdlStateMachine（ROS 节点）
//                    —— 只做三件事：等依赖就绪 → 调用发送函数 → 把结果翻译成下一个状态；
//                       **不定义任务链条**（发几条、什么顺序，交给 TaskSource 决定）。
//
// ===========================================================================
// 状态图（四个状态，只有一个涉及任务）
// ===========================================================================
//   kIdle ─(auto_start / ~/start)─▶ kWaitForSystem ─▶ kSendTask ─┬─▶ kSendTask（来源还有下一条）
//                                        │ 依赖超时              │
//                                        ▼                       ├─▶ kFinished（来源说没有了）
//                                     kFailed ◀──────────────────┴─ 任一条任务失败
//
//   kSendTask 每拍（默认 50 ms）二选一：
//     没有在飞的任务 → 向 TaskSource 要一条目标 → sendTask()
//     有在飞的任务   → pollTask()：kRunning 就继续等；kReady 则按 success 决定
//                      "再要下一条" 还是 "进 kFailed"
//
// ===========================================================================
// 为什么"发一条任务"是 sendTask() + pollTask() 两个函数
// ===========================================================================
//   /control_task 的语义是"把这条轨迹执行完再返回"（几秒），所以用**异步**调用：
//   sendTask() 只负责发出并立刻返回，pollTask() 每拍查一次结果。
//   若做成一个阻塞的 sendTaskAndWait()，调用者（例如状态机的定时器线程）会被占住几秒，
//   期间收不到 `~/stop`、也发不出 `~/state`。
//   句柄正是为此存在：它把"有一条在飞"的记账从函数里搬到调用者手里，
//   这样函数本身可以是无状态的自由函数。
//
// ===========================================================================
// 用法 A：只要发送一条任务（不需要状态机）
// ===========================================================================
//   auto node = std::make_shared<rclcpp::Node>("my_client");
//   kdl_state_machine::TaskHandle handle;
//   handle.client = node->create_client<kdl_tools::srv::ControlTask>("/control_task");
//
//   kdl_state_machine::TaskTarget target;          // ← 目标来自外部，不是内部状态
//   target.kind = kdl_state_machine::TaskKind::kJointSpace;
//   target.joint_goal = {0.0, -0.6, 0.8, 0.0, 0.2, 0.0};
//   target.duration = 8.0;                         // <= 0 交给 control 层自动定时
//
//   std::string message;
//   if (kdl_state_machine::sendTask(handle, target, message) != SendStatus::kOk) { ... }
//   kdl_state_machine::TaskResult result;
//   while (kdl_state_machine::pollTask(handle, result) == PollStatus::kRunning) {
//     rclcpp::spin_some(node);                     // 或别的等待方式
//   }
//   if (!result.success) { /* 看 result.error_code + result.message */ }
//
// ===========================================================================
// 用法 B：要"什么时候发、发几条"由状态机调度
// ===========================================================================
//   语法糖（推荐）：startJointTask() / startCartesianTask() 直接指定一条任务 ——
//     node->startJointTask({0.5, 0.5, 0.6, 0.7, 0.3, 0.0, 6.0});        // 末位是时长 [s]
//     node->startCartesianTask({0.908, 0.016, 0.594, 0.742, -0.072, -0.662, -0.081, 8.0});
//   统一入口：startTask(kind, values, message) —— 新增任务类型也从这里进来。
//   再跑一遍：ros2 service call /state_machine/start std_srvs/srv/Trigger {}
//   多条任务的链条：setTaskSource() 注入自己的来源（见"扩展点 ②"），状态机不掺和
//
// ===========================================================================
// 扩展点
// ===========================================================================
//   ① 新任务类型（关节空间 / 笛卡尔空间之外）：四步 ——
//        a) TaskKind 里加枚举值（下面有注释占位）；
//        b) TaskTarget 里补该任务需要的目标字段；
//        c) buildTarget() 里加一个 case（数值数组 → 目标，src/kdl_state_machine.cpp）；
//        d) buildRequest() 里加一个 case（目标 → service 请求，同文件）。
//      状态机**一行都不用改**（它只认识 TaskTarget）。
//   ② 任务链条 / 作业流程：setTaskSource() 注入来源（读 YAML、查表、看传感器都行），
//      状态机只负责"向来源要一条、发出去、等结果"。
//
// ===========================================================================
// 对外接口与参数（用法 B）
// ===========================================================================
//   srv  ~/start   std_srvs/Trigger  从头开始跑（终态或空闲时才接受）
//   srv  ~/stop    std_srvs/Trigger  停止推进后续步骤（不取消已下发的任务）
//   pub  ~/state   std_msgs/String   当前状态名（failed 时附带失败原因）
//   参数（见 src/kdl_state_machine.cpp 的 readParameters）：
//     service_name / status_topic / joint_states_topic /
//     auto_start / tick_period / system_timeout / start_delay
// ===========================================================================

#ifndef KDL_STATE_MACHINE__KDL_STATE_MACHINE_HPP_
#define KDL_STATE_MACHINE__KDL_STATE_MACHINE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "kdl_tools/msg/control_status.hpp"
#include "kdl_tools/srv/control_task.hpp"

namespace kdl_state_machine
{

// ===========================================================================
// 一、任务的定义：要做什么 / 结果如何
// ===========================================================================

/**
 * @brief 任务类型：决定 TaskTarget 里哪些字段有效、请求怎么构造。
 *
 * @note 新增任务类型时：① 这里加枚举值；② TaskTarget 补字段；③ buildRequest() 加 case。
 */
enum class TaskKind
{
  kJointSpace,      ///< 关节空间：给目标关节角（TaskTarget::joint_goal）
  kCartesianSpace   ///< 笛卡尔空间：给目标末端位姿（TaskTarget::pose）

  // ---- 预留的任务类型（后续扩展时取消注释，并同步上面三步）----
  // kCircular,     ///< 圆弧：需要 途经点/终点 或 圆心+半径 等参数
  // kSpline,       ///< 样条：需要路点数组 + 每段时长
  // kForce,        ///< 力控 / 阻抗：需要期望力（力矩）+ 施力方向 + 选择矩阵
  // kGripper,      ///< 夹爪等非运动类任务：需要开合度等参数
};

/**
 * @brief 任务类型的可读名字，用于日志与打印。
 * @param kind [in] 任务类型。
 * @return 常量字符串（"joint_space" / "cartesian_space"）；未识别时返回 "unknown"。
 */
const char * taskKindName(TaskKind kind);

/**
 * @brief 一条任务的目标：**只描述"要做什么"，不含"现在走到哪一步"**。
 *
 * @note 哪些字段有效由 kind 决定，其余字段被忽略；因此不需要联合体式的严格约束，
 *       新增任务类型时补字段即可。
 */
struct TaskTarget
{
  /// 任务类型。
  TaskKind kind = TaskKind::kJointSpace;

  /// kJointSpace 用：目标关节角 [rad]，长度必须等于机器人关节数（由 control 层校验）。
  std::vector<double> joint_goal{};

  /// kCartesianSpace 用：目标末端位姿，相对基座。四元数不必归一化（control 层会归一）。
  geometry_msgs::msg::Pose pose{};

  /// 轨迹时长 [s]；<= 0 表示交给 control 层按限位自动定时。
  double duration = 0.0;

  // ---- 预留：新任务类型需要的目标字段（例如圆弧的途经点、样条的路点数组）----
};

/**
 * @brief 一次任务的结果：service 响应的精简版。
 */
struct TaskResult
{
  bool success = false;             ///< = (error_code == 0 && success)
  int32_t error_code = 0;           ///< 见 ControlTask.srv 的三段语义
  std::string message{};            ///< 中文说明（成功摘要 / 失败原因）
  double trajectory_duration = 0.0;  ///< 实际下发的轨迹时长 [s]
};

// ===========================================================================
// 二、任务发送：自由函数 + 调用者持有的句柄
// ===========================================================================

/**
 * @brief 任务收发的句柄：**由调用者持有**（状态机、脚本、别的节点都行）。
 *
 * @note 之所以要有它：service 是"执行完才返回"的，异步发送必须记住"有一条在飞"。
 *       把这记账放在句柄里，发送函数就能保持**无状态**（不依赖任何状态机的内部状态）。
 * @note 一个句柄同时只承载一条任务（sendTask 时若上一条还在飞会返回 kBusy）；
 *       想并发就各自持有自己的句柄。
 */
struct TaskHandle
{
  /// `/control_task` 的客户端（由调用者按自己的节点创建；为空时 sendTask 返回 kNoService）。
  rclcpp::Client<kdl_tools::srv::ControlTask>::SharedPtr client{};

  /// 在飞的请求（sendTask 之后、pollTask 取走结果之前有效）。调用者不必直接读写它。
  std::shared_future<kdl_tools::srv::ControlTask::Response::SharedPtr> pending{};
};

/// sendTask() 的结果。
enum class SendStatus
{
  kOk,          ///< 请求已发出（异步，结果要用 pollTask 取）
  kBusy,        ///< 句柄里已经有一条任务在飞
  kBadTarget,   ///< 目标非法（例如 joint_space 却没给 joint_goal）
  kNoService    ///< service 不可用（client 为空，或 /control_task 还没起来）
};

/// pollTask() 的结果。
enum class PollStatus
{
  kIdle,     ///< 没有在飞的任务（还没 send，或上次结果已被取走）
  kRunning,  ///< 有任务在飞，还没拿到结果
  kReady     ///< 结果到了（成功与否看 TaskResult::success）
};

/**
 * @brief 把 TaskTarget 翻译成 ControlTask.srv 的请求（**新增任务类型的唯一落点**）。
 * @param target  [in]  任务目标。
 * @param request [out] 构造好的请求（函数内先清空，避免残留上次的字段）。
 * @param message [out] 失败原因（中文）。
 * @return true 表示目标合法、请求已填好。
 *
 * @note 任务类型的分派就在这里：当前只有 kJointSpace / kCartesianSpace 两个 case，
 *       新增类型时加 case（见 TaskKind 的注释占位）。它不碰 service，可单独调用与测试。
 * @note 它不校验"目标是否在关节行程内 / 工作空间内"：那是 control 层的职责，
 *       会以 error_code = 3 + 中文原因返回。
 */
bool buildRequest(
  const TaskTarget & target, kdl_tools::srv::ControlTask::Request & request,
  std::string & message);

/**
 * @brief 把"目标 + 时长"打包的数值数组翻译成 TaskTarget（**数组约定只在这一处**）。
 * @param kind    [in]  任务类型（决定 values 怎么解释）。
 * @param values  [in]  数值数组，**末位固定是时长 [s]**：
 *                      - kJointSpace     : [q1, q2, ..., qn, duration]
 *                      - kCartesianSpace : [x, y, z, qx, qy, qz, qw, duration]
 * @param target  [out] 填好的任务目标（函数内先清空，避免残留上次的字段）。
 * @param message [out] 失败原因（中文）。
 * @return true 表示数组合法、目标已填好。
 *
 * @note 时长 <= 0 表示交给 control 层按限位自动定时（与 TaskTarget::duration 约定一致）。
 * @note **长度不参与类型推断**：关节数恰好可能让长度撞上 8（例如 7 轴 + 时长），
 *       所以类型必须显式给（kind），不能靠"猜长度"。
 * @note 它不校验"目标是否在关节行程内 / 工作空间内"：那是 control 层的职责，
 *       会以 error_code = 3 + 中文原因返回。
 * @note 新增任务类型时在这里加 case（与 TaskKind / buildRequest 对齐）。
 */
bool buildTarget(
  TaskKind kind, const std::vector<double> & values, TaskTarget & target, std::string & message);

/**
 * @brief **下发一条任务**（异步，立刻返回，不阻塞）。
 * @param handle  [in,out] 收发句柄（调用者持有）。
 * @param target  [in]     任务目标（完全来自外部，本函数不读任何内部状态）。
 * @param message [out]    失败原因（中文）。
 * @return kOk 表示已发出；其余见 SendStatus。
 *
 * @note 顺序：先查"上一条是否还在飞" → 再翻译目标 → 再看 service 是否可用 → 才发出。
 *       目标非法时不会因为"服务没起来"而报成 kNoService。
 * @note 发出后要用 pollTask() 取结果；只发不收会让句柄一直是"忙"。
 */
SendStatus sendTask(TaskHandle & handle, const TaskTarget & target, std::string & message);

/**
 * @brief 取一次结果（非阻塞，配合 sendTask 使用）。
 * @param handle [in,out] 收发句柄。
 * @param result [out]    仅在返回 kReady 时被填充。
 * @return kIdle = 没有在飞的任务；kRunning = 还没好；kReady = 结果已取走（result 有效）。
 *
 * @note 取走结果后句柄复位，下一次 pollTask() 返回 kIdle，直到再次 sendTask()。
 */
PollStatus pollTask(TaskHandle & handle, TaskResult & result);

/**
 * @brief 丢掉等待：不再关心在飞任务的结果（句柄立刻变回"空闲"）。
 * @param handle [in,out] 收发句柄。
 *
 * @note **不会取消任务**：请求已经发出，服务端（以及机械臂）会照常把轨迹执行完；
 *       本函数只让调用者"不再等"，典型用途是状态机的 `~/stop`。
 */
void discardTask(TaskHandle & handle);

/**
 * @brief 句柄里是否有一条任务在飞。
 * @param handle [in] 收发句柄。
 * @return true 表示 sendTask 之后、pollTask 取走结果之前。
 */
bool taskInFlight(const TaskHandle & handle);

/**
 * @brief `/control_task` 是否可用（client 存在且已被发现）。
 * @param handle [in] 收发句柄。
 * @return true 表示可以 sendTask。
 */
bool serviceReady(const TaskHandle & handle);

// ===========================================================================
// 三、任务状态机：只负责"什么时候发、发几条"，不定义链条
// ===========================================================================

/**
 * @brief 状态机的状态。每个状态在 tick() 里对应一个同名处理函数。
 *
 * @note 迁移路径固定为：
 *       kIdle → kWaitForSystem → kSendTask（可自环，来源决定发几条）→ kFinished；
 *       任何一步失败都直接跳到 kFailed，并把原因记进 fail_message_。
 * @note kFinished / kFailed 是**终态**：停在那里不再自动推进，调 `~/start` 可以清掉
 *       上一次的残留、从头再跑一遍（见 resetRun()）。
 * @note 这里**没有**"具体任务"的状态（例如"到预备位形""末端内收"）：任务类型由
 *       TaskKind 表达、"发几条/什么顺序"由 TaskSource 决定，状态机不跟着作业流程改。
 */
enum class State
{
  kIdle,           ///< 未启动；等 auto_start（只生效一次）或 `~/start`
  kWaitForSystem,  ///< 等依赖就绪（服务可见 + 控制器状态 + /joint_states，再等 start_delay）
  kSendTask,       ///< 调任务发送函数：向 TaskSource 要一条 → sendTask() → pollTask()
  kFinished,       ///< 任务来源说"没有更多任务"（终态）
  kFailed          ///< 任一条任务失败（终态，fail_message_ 说明原因）
};

/**
 * @brief 状态 → 可读名字，用于日志与 `~/state` 话题。
 * @param state [in] 状态值。
 * @return 常量字符串（"idle" / "wait_for_system" / "send_task" / "finished" / "failed"），
 *         不会返回 nullptr。
 *
 * @note 这些名字就是 `~/state` 话题里发出去的内容，属于**对外契约**：改名等于改接口。
 */
const char * stateName(State state);

/**
 * @brief 任务级状态机（ROS 节点）：任务发送委托给 sendTask/pollTask，自己只管调度。
 *
 * 它不认识 KDL 轨迹、不认识具体任务类型（只认识 TaskTarget）、也不定义任务链条。
 *
 * @note 对外接口：
 *       - srv `~/start`（std_srvs/Trigger）：终态或空闲时接受，从头跑一遍；
 *       - srv `~/stop`（std_srvs/Trigger）：停止推进后续步骤（不取消已下发的任务）；
 *       - pub `~/state`（std_msgs/String，transient_local）：当前状态名，失败时带原因。
 * @note 单飞行：同一时刻只允许一条任务在飞（由 TaskHandle 保证；control 层也不支持
 *       并发，第二条请求会被它回 error_code=7）。
 * @note "依赖就绪"必须**同时**满足三件事：/control_task 服务可见、收到控制器状态、
 *       收到 /joint_states。三者缺一不可 —— 仿真里 joint_state_broadcaster 与力矩
 *       控制器是两个 spawner **并行**加载的，可能控制器已经 active 而 /joint_states
 *       还没发出来，此时下发任务会被 control 节点回 error_code = 1（实测踩过这个
 *       竞态）；依赖凑齐后再多等 start_delay 才发第一条任务，避开"control 节点
 *       还没处理完第一帧状态"的窄窗口。
 * @note 状态机由**墙钟**定时器驱动（不是仿真时钟）：仿真被 ~/set_pause 暂停时仍能
 *       观察到"等依赖超时"、仍能响应 `~/stop`，不会跟着仿真时钟一起停摆。
 * @note 所有回调（定时器、两个 srv、status 与 joint_states 订阅）都在默认的互斥
 *       回调组里，彼此串行，因此 state_ 等运行期成员不需要加锁。
 */
class KdlStateMachine : public rclcpp::Node
{
public:
  using ControlTask = kdl_tools::srv::ControlTask;
  using ControlStatus = kdl_tools::msg::ControlStatus;
  using Trigger = std_srvs::srv::Trigger;

  /**
   * @brief 任务来源：状态机每次要发任务时向它要一条目标。
   * @param target [out] 被填好的任务目标。
   * @return true = 有任务（target 有效）；false = 没有更多任务（状态机进 kFinished）。
   *
   * @note **这就是"任务链条"的接口**：链条怎么排、目标从哪来、什么时候结束，
   *       全在实现里；状态机自己不产生任何目标。
   */
  using TaskSource = std::function<bool(TaskTarget & target)>;

  /**
   * @brief 构造即完成装配：读参数 → 建接口（含 TaskHandle）。
   * @param options [in] rclcpp 节点选项（launch 传进来的节点名、remap 会在这里生效）。
   *
   * @note 装配失败会抛出 std::runtime_error（message 里是中文原因），由 main() 捕获退出
   *       —— 宁可在 launch 里立刻看到失败，也不要带着半成品状态继续跑。
   * @note 构造函数**不装任何任务来源**：启动前请用 startJointTask() / startCartesianTask()
   *       （或 startTask()）指定一条任务，或用 setTaskSource() 注入自己的链条；
   *       两者都没设时，状态机在 kSendTask 会因"没有任务来源"直接进 kFinished。
   * @note 构造函数末尾会发一次 `~/state`（transient_local），晚订阅的观察者也能看到初值。
   */
  explicit KdlStateMachine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 注入自定义任务来源，覆盖默认的"只发参数里那一条"。
   * @param source [in] 任务来源；传空 std::function 表示"没有任务来源"（直接进 kFinished）。
   *
   * @note 这是状态机唯一的扩展点。最好在 spin 之前设置：auto_start 为 true 时，
   *       依赖就绪后的第一拍就会来要任务。
   * @note 在运行中替换也允许，但只在"下一次要新任务"时生效（正在飞的那条不会被打断）。
   */
  void setTaskSource(TaskSource source);

  /**
   * @brief **启动一条关节空间任务**（数值数组版；最终走 startTask()）。
   * @param values  [in]  `[q1, q2, ..., qn, duration]`，**末位是时长 [s]**（<= 0 自动定时）。
   * @param message [out] 失败原因（中文）。
   * @return true = 已接受并开始跑；false = 数组非法 / 状态机正在运行。
   *
   * @note 等价于 `startTask(TaskKind::kJointSpace, values, message)`。
   */
  bool startJointTask(const std::vector<double> & values, std::string & message);

  /**
   * @brief **启动一条笛卡尔空间任务**（数值数组版；最终走 startTask()）。
   * @param values  [in]  `[x, y, z, qx, qy, qz, qw, duration]`，**末位是时长 [s]**。
   * @param message [out] 失败原因（中文）。
   * @return true = 已接受并开始跑；false = 数组非法 / 状态机正在运行。
   *
   * @note 等价于 `startTask(TaskKind::kCartesianSpace, values, message)`。
   * @note 起点是下发时刻的 FK(q_now)：当前位形若接近奇异（例如零位的竖直直臂），
   *       control 层会拒绝该任务（error_code = 3）。
   */
  bool startCartesianTask(const std::vector<double> & values, std::string & message);

  /**
   * @brief **启动一条指定模式的任务**（统一入口；新增任务类型也从这里进来）。
   * @param kind    [in]  任务类型（决定 values 怎么解释，见 buildTarget()）。
   * @param values  [in]  数值数组，**末位固定是时长 [s]**。
   * @param message [out] 失败原因（中文）。
   * @return true = 已接受并开始跑；false = 数组非法 / 状态机正在运行。
   *
   * @note 接受条件与 `~/start` 一致：只在 kIdle / kFinished / kFailed 接受；
   *       运行中调用会被拒（false），避免中途替换任务导致参考跳变。
   * @note 它把任务来源换成"只发这一条"的实现，**覆盖**之前 setTaskSource() 注入的链条
   *       （反之亦然，后设的生效）。
   * @note 本轮跑完（kFinished）后调 `~/start` 可以再跑一遍同一条任务
   *       （resetRun() 会清掉"已给过"的标记）。
   * @note 线程：与节点内其它回调同属默认互斥回调组，请在 spin 之前、或在该回调组内
   *       （例如某个 service 回调里）调用，不要从任意线程调。
   */
  bool startTask(TaskKind kind, const std::vector<double> & values, std::string & message);

private:
  // -------------------------------------------------------------------------
  // 状态处理函数：一个状态一个函数
  // -------------------------------------------------------------------------

  /**
   * @brief 状态 kIdle：等启动信号。
   * @return auto_start 首次生效时返回 kWaitForSystem；否则一直是 kIdle。
   *
   * @note auto_start 只生效一次（auto_started_）：这样 `~/stop` 把状态机打回 idle 之后，
   *       不会被它立刻又拉起来，停止才是真的停止。
   */
  State stepIdle();

  /**
   * @brief 状态 kWaitForSystem：等三项依赖就绪，再等 start_delay 才放行。
   * @return 依赖齐了且已安定够 start_delay → kSendTask；否则留在本状态；
   *         超过 system_timeout 仍未就绪 → kFailed（message 指出缺哪一项）。
   *
   * @note 依赖中途掉线（例如广播器重启）会**重新计时**，不能让"曾经凑齐过"一直成立。
   * @note 时限用墙钟计（steady_clock），与 kdl_control_node 的超时口径一致。
   */
  State stepWaitForSystem();

  /**
   * @brief 状态 kSendTask：调任务发送函数——向 TaskSource 要一条，发出，然后收结果。
   * @return 在飞时留在本状态；任务成功 → kSendTask（继续向来源要下一条）；
   *         来源说没有更多 → kFinished；任一步失败 → kFailed。
   *
   * @note 本函数是状态机里**唯一**涉及任务的代码，且它只做"要目标、调用发送函数、
   *       翻译结果"，不含任何任务类型判断（TaskTarget 是唯一的数据形态）。
   */
  State stepSendTask();

  /**
   * @brief 记录失败原因并把状态机置为 kFailed（不抛异常，与底层模块的失败风格一致）。
   * @param message [in] 失败原因（中文，会被原样发到 `~/state`）。
   * @return 恒为 State::kFailed，方便写 `return fail(message);`。
   *
   * @note 原因同时留在 fail_message_ 里（供 `~/state` 与 `~/start` 后的复位使用），
   *       并用 ERROR 级别打一次日志。
   */
  State fail(const std::string & message);

  /**
   * @brief 定时器回调：按当前状态下调一次对应的处理函数，并完成状态迁移。
   *
   * @note 每 tick_period_ 秒（默认 50 ms）被调一次；kFinished / kFailed 直接返回
   *       （终态等 `~/start`）。
   * @note 处理函数返回的是"下一个状态"，统一交给 setState() 打日志并发布 `~/state`，
   *       所以状态迁移只有 setState() 这一个出口。
   */
  void tick();

  /**
   * @brief 状态迁移：同值不动作；不同值则打 INFO 日志 + 发布 `~/state`。
   * @param next [in] 迁移到的目标状态。
   */
  void setState(State next);

  /**
   * @brief 把当前状态发布到 `~/state`。
   *
   * @note 状态为 kFailed 时把 fail_message_ 一起带上，订阅者不必再翻日志。
   * @note QoS 是 keep_last(1) + transient_local：晚订阅的观察者也能立刻看到当前状态。
   */
  void publishState();

  /**
   * @brief 清掉上一次运行的残留，准备重跑（`~/start` 的调用点）。
   *
   * @note 清 fail_message_ / 计数 / 等待计时（含 all_ready_）/ 默认来源的"已用过"标记；
   *       **不清** status_seen_ 与 joint_states_seen_ —— 收到过就是收到过，依赖还在；
   *       也**不动**注入的 task_source_（自定义来源自己负责重跑语义）。
   */
  void resetRun();

  // -------------------------------------------------------------------------
  // 装配（构造期）
  // -------------------------------------------------------------------------

  /**
   * @brief 读参数（含默认值）并做合法性校验。
   * @param message [out] 失败原因（中文）。
   * @return true 表示全部参数合法。
   *
   * @note 参数与默认值见 .cpp；校验覆盖 tick_period > 0、system_timeout > 0、
   *       start_delay >= 0、task_duration 有限、task_kind 可识别、task_pose 长度为 7。
   */
  bool readParameters(std::string & message);

  /**
   * @brief 是否可以从头开始跑一轮（只允许 kIdle / kFinished / kFailed）。
   * @param message [out] 不允许时的原因（中文）。
   * @return true 表示可以开始。
   */
  bool canStart(std::string & message) const;

  /**
   * @brief 开始新的一轮：resetRun() + 进 kWaitForSystem（`~/start` 与 startTask() 共用）。
   * @param message [out] 不允许时的原因（中文）。
   * @return true 表示已启动。
   */
  bool beginRun(std::string & message);

  /**
   * @brief 建接口：service client + TaskHandle、`~/state` 发布、两个订阅、两个 srv、定时器。
   * @param message [out] 失败原因（中文）。
   * @return true 表示全部创建成功。
   *
   * @note 这里**不等待**依赖可用（那是 stepWaitForSystem() 的事）：构造期要快速失败，
   *       不能把节点启动堵住。
   */
  bool setupInterfaces(std::string & message);

  /**
   * @brief 启动横幅：把参数、默认任务、以及"任务来源是谁"打印出来。
   *
   * @note 配置改了要能一眼看出来，所以默认任务的目标也打出来。
   */
  void printPlan() const;

  // -------------------------------------------------------------------------
  // 回调
  // -------------------------------------------------------------------------

  /**
   * @brief 控制器状态回调：只用来把 status_seen_ 置起来。
   * @param msg [in] 控制器状态（/kdl_effort_controller/status）。
   *
   * @note 控制器只在 update() 被调用（= 已激活）之后才会发这个话题，所以"收到过一帧"
   *       就是"控制器已经 active"的充分证据；之后的消息直接忽略，不刷日志。
   */
  void statusCallback(const ControlStatus::SharedPtr msg);

  /**
   * @brief /joint_states 回调：只用来把 joint_states_seen_ 置起来。
   * @param msg [in] 关节状态（判断状态广播器是否已 active）。
   *
   * @note 必须查它，不能只看控制器状态：joint_state_broadcaster 与 kdl_effort_controller
   *       是两个 spawner 并行加载的，控制器可能先 active，此时下发任务会被 control 节点
   *       回 error_code = 1（实测踩过）。
   */
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief `~/start`：清掉上次残留，从头跑一遍。
   * @param request  [in]  空请求（std_srvs/Trigger）。
   * @param response [out] success = 是否接受；message 说明原因或当前状态。
   *
   * @note 只在 kIdle / kFinished / kFailed 接受；运行中调用会被拒（success = false），
   *       避免中途替换任务导致参考跳变。
   */
  void startCallback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  /**
   * @brief `~/stop`：停止推进后续步骤（状态回到 kIdle）。
   * @param request  [in]  空请求（std_srvs/Trigger）。
   * @param response [out] success = 是否真的停了；message 说明原因。
   *
   * @note **不取消已经下发的那条任务**：discardTask() 只是丢掉等待，那条轨迹仍会被
   *       控制器跑完。要再次启动的话，最好等它跑完（否则新任务会被 control 层回
   *       error_code = 7 "已有任务在执行"）。
   * @note 已经在终态时返回 success = false（"本来就没在跑"）。
   */
  void stopCallback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  // ---- 参数（构造期读一次，之后只读）----
  std::string service_name_{};        ///< 任务下发服务，默认 /control_task
  std::string status_topic_{};        ///< 控制器状态话题，默认 /kdl_effort_controller/status
  std::string joint_states_topic_{};  ///< 关节状态话题，默认 /joint_states（依赖判据之一）
  bool auto_start_ = true;            ///< 构造后是否自动开始跑
  double tick_period_ = 0.05;         ///< 状态机节拍 [s]（墙钟）
  double system_timeout_ = 60.0;      ///< 等依赖就绪的上限 [s]（墙钟）
  double start_delay_ = 0.5;          ///< 依赖凑齐后再等多久才发第一条任务 [s]

  // ---- 任务发送（自由函数 + 本句柄；见本文件第二节）----
  TaskHandle handle_{};          ///< service client 与"有一条在飞"的记账
  TaskSource task_source_{};     ///< 任务来源（startTask / setTaskSource 设置；默认空）
  bool one_shot_used_ = false;   ///< startTask() 装的"只发这一条"是否已给过（resetRun 复位）

  // ---- 运行期 ----
  State state_ = State::kIdle;  ///< 当前状态（迁移只走 setState()）
  std::string fail_message_{};  ///< kFailed 的原因，随 `~/state` 一起发布
  bool auto_started_ = false;   ///< auto_start 是否已经生效过（免得 ~/stop 后被立刻拉起来）
  bool status_seen_ = false;    ///< 是否收到过控制器状态（= 控制器已激活）
  bool joint_states_seen_ = false;  ///< 是否收到过 /joint_states（= 状态广播器已 active）
  bool wait_started_ = false;   ///< 是否已经开始计"等依赖"的时间
  std::chrono::steady_clock::time_point wait_start_{};  ///< 开始等依赖的时刻（超时基准）
  bool all_ready_ = false;      ///< 三项依赖是否已凑齐（凑齐后才开始计 start_delay）
  std::chrono::steady_clock::time_point all_ready_at_{};  ///< 依赖凑齐的时刻（安定计时基准）
  unsigned int completed_steps_ = 0;  ///< 已成功完成的任务条数（横幅里打印）

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;  ///< `~/state` 发布
  rclcpp::Subscription<ControlStatus>::SharedPtr status_sub_;      ///< 控制器状态订阅
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;  ///< 关节状态订阅
  rclcpp::Service<Trigger>::SharedPtr start_srv_;                  ///< `~/start`
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;                   ///< `~/stop`
  rclcpp::TimerBase::SharedPtr tick_timer_;                        ///< 墙钟定时器（状态机节拍）
};

}  // namespace kdl_state_machine

#endif  // KDL_STATE_MACHINE__KDL_STATE_MACHINE_HPP_
