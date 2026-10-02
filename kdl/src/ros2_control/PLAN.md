# P5 实现计划：control → MuJoCo 桥接

> ⚠️ **本文档已被 [`PLAN_TORQUE.md`](./PLAN_TORQUE.md) 取代（仅作历史记录保留）。**
> 这里描述的是**第一版实现**：`control_bridge_node` + `arm_controller`(JTC) + `position` 命令接口。
> 该链路已退休，取而代之的是「`kdl_control_node` + `kdl_effort_controller`（effort 接口 + 力矩控制）」，
> 现状请看 [`PLAN_TORQUE.md`](./PLAN_TORQUE.md)、[`力矩控制问题报告.md`](./力矩控制问题报告.md)
> 与 [`README.md`](./README.md)。下面内容中的接口名/控制器名/命令接口均已过时，**不要照着实现**。

> 需求来源：`src/control/PLAN.md` 的 P5 阶段。
> 本文件是该需求的落地设计：接口定义、节点结构、关键实现细节、构建/安装接线、验收标准与风险。
> **状态：已实现并实测通过**（见 §11 实现记录；偏差以 §11 为准）。

---

## 0. 需求原文与对应关系

| `src/control/PLAN.md` P5 的要求 | 本计划的落实位置 |
|---|---|
| 与 `src/ros2_control` 桥接 | §1 目标与边界 |
| 把 `ControlResult` 采样成 `trajectory_msgs/JointTrajectory` | §4.2 轨迹转换 |
| 发给 `arm_controller` | §4.2 轨迹转换 |
| 在 MuJoCo 里 `Goal finished with status: SUCCEEDED` | §8 验收标准 |

---

## 1. 目标与边界

**目标**：写一个 ROS 2 节点，把 `TaskRouter` 的输出接到 MuJoCo 硬件上执行。

**链路**：

```
调用者 → Service(/control_task) → ControlBridgeNode → TaskRouter::dispatch()
                                                      ↓
                                            ControlResult::joint_trajectory
                                                      ↓
                                          采样成 JointTrajectory（point_dt 网格）
                                                      ↓
                                    Action(/arm_controller/follow_joint_trajectory)
                                                      ↓
                                              MuJoCo 执行（JTC 写位置命令）
```

**三条硬边界**：

1. **不修改 `src/control/` 的任何代码**：control 保持"只依赖 KDL + rclcpp"，不引入 ROS 消息类型。
2. **不修改 `src/ros2_control/` 的硬件配置**：controllers.yaml、xacro、MJCF 等都不动。
3. **只新增文件 + 修改 CMake + 修改 launch**：桥接逻辑全部放在新节点里。

**明确不做的事**（本期）：多段任务、力矩前馈在线计算、任务取消、执行进度反馈。

---

## 2. 接口设计

### 2.1 Service 定义

**Service: `/control_task`**，文件 `src/communication/srv/ControlTask.srv`。

> **它属于 `kdl_tools` 这个包**：`src/communication/` 是与 `Tools`、`control` 平级的
> "通信层"目录（只有一个 `srv/` 子目录，没有 `package.xml`/`CMakeLists.txt`），
> 所以它由 `kdl_tools` 自己用 `rosidl_generate_interfaces` 生成，而不是一个独立包。
> 将来若有别的包要消费这个接口，再把它拆成独立的 interface 包（那时才需要
> `package.xml` + `CMakeLists.txt` + 独立的构建顺序）。

```
# 请求
uint8 JOINT_SPACE = 0               # 关节空间任务
uint8 CARTESIAN_SPACE = 1           # 笛卡尔空间任务

uint8 task_type
float64[] goal_joint                # JOINT_SPACE: 目标关节角 [rad]，长度 = 关节数
geometry_msgs/Pose goal_pose        # CARTESIAN_SPACE: 目标位姿（相对基座）
float64 duration                    # [s]，<=0 自动估算（不短于 min_duration）
---
# 响应
int32 error_code                    # 0 成功；1..7 桥接层错误；<0 = JTC 的 error_code
bool success                        # = (error_code == 0)
string message                      # 中文说明（成功摘要 / 失败原因）
float64 trajectory_duration         # 实际下发的轨迹时长 [s]
```

**为什么响应里必须有 `error_code`**（原计划只有 `success/message/trajectory_duration`）：
原计划 §2.2 说"执行结果由 arm_controller 的 action 反馈返回"，但调用者拿不到那个
反馈 —— 它只拿到 service 的响应。没有 `error_code` 时，"解算失败"和"JTC 判路径容差
超限（-4）"在调用者眼里都是 `success=false`，验收标准 2（"MuJoCo 执行 → 到位"）也就
无从判定。错误码分三段：`0` 成功；`1..7` 是桥接层自己的失败（1 无 joint_states、
2 请求非法、3 解算失败、4 action 不可用、5 goal 被拒、6 超时、7 忙）；`<0` 直接透传
JTC 的 `error_code`（-1 目标非法、-2 关节不匹配、-3 时间戳过期、-4 路径容差、-5 终点容差）。

### 2.2 为什么用 Service 而不是 Action

- 任务解算本身很快（< 100 ms），不需要异步
- 执行结果由 arm_controller 的 action 负责，桥接节点只负责"解算 + 下发 + 回报结果"
- 教学项目，简洁优先

### 2.3 为什么不用 Topic

- Topic 没有响应，调用者不知道解算是否成功
- Service 的"请求-响应"语义更清晰

---

## 3. 节点结构

**`ControlBridgeNode`**（新建 `src/ros2_control/src/control_bridge_node.cpp`）

```cpp
class ControlBridgeNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;  // 注意 rclcpp_action
  using ControlTask = kdl_tools::srv::ControlTask;

  ControlBridgeNode();

private:
  // 构造时完成：
  // 1. 参数（urdf_file / control_action / joint_names / 限位 / point_dt / min_duration）
  // 2. 从 URDF 装配 TaskRouter（loadContextFromUrdf，并**自己补上** joint_limits）
  // 3. 建立"控制器关节名 ↔ KDL 链关节下标"的映射
  // 4. 订阅 /joint_states、创建 service /control_task、建 action client

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlTaskCallback(const std::shared_ptr<ControlTask::Request> request,
                           std::shared_ptr<ControlTask::Response> response);

  bool snapshotState(KDL::JntArray & q, KDL::JntArray & qdot, std::string & message);
  bool buildTaskRequest(const ControlTask::Request & request,
                        kdl_control::TaskRequest & task, std::string & message);
  bool buildTrajectory(const kdl_control::ControlResult & result,
                       trajectory_msgs::msg::JointTrajectory & trajectory, std::string & message);
  void sendAndWait(const trajectory_msgs::msg::JointTrajectory & trajectory,
                   double trajectory_duration,
                   std::shared_ptr<ControlTask::Response> response);

  kdl_control::TaskRouter router_;
  std::vector<std::string> controller_joints_;                  // 消息里的顺序（= controllers.yaml）
  std::vector<std::string> chain_joints_;                       // 链的顺序（= 解算输出顺序）
  std::vector<unsigned int> chain_index_of_controller_joint_;   // 两者按名字对齐

  std::unordered_map<std::string, std::pair<double, double>> state_;  // 关节名 -> (q, qdot)
  bool state_ready_ = false;
  std::mutex state_mutex_;   // 保护 state_ / state_ready_
  std::mutex task_mutex_;    // 同一时刻只允许一条任务在飞
  ...
};
```

---

## 4. 关键实现细节

### 4.1 当前状态获取

**问题 1**：`/joint_states` 的关节顺序可能与链的顺序不同 → 按关节名映射，不依赖顺序。

**问题 2（原计划的代码是错的）**：不能这样取链上的关节名：

```cpp
// ✗ 错：一旦链里有"没有关节的段"（固定连接），关节数 != 段数，整个映射会错位
for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
  chain.getSegment(i).getJoint().getName();
}
```

正确写法是遍历**段**并跳过 `KDL::Joint::None`：

```cpp
for (unsigned int seg = 0; seg < chain.getNrOfSegments(); ++seg) {
  const KDL::Joint & joint = chain.getSegment(seg).getJoint();
  if (joint.getType() == KDL::Joint::None) { continue; }
  chain_joints_.emplace_back(joint.getName());
}
```

**问题 3（原计划没有）**：必须有一个"状态是否就绪"的显式标记。回调里只要出现
缺关节 / 非有限值，就把 `state_ready_` 置回 `false`；service 拿不到完整状态时返回
`error_code = 1`，而不是拿半截状态去解算。

**问题 4（原计划没有）**：`/joint_states` 的 **QoS**。`joint_state_broadcaster` 用的是
`SystemDefaultsQoS`（reliable），本节点订阅用 `SensorDataQoS`（best effort）——
best-effort 订阅者与 reliable 发布者是兼容的，反过来不保证，所以取更宽松的那个，
换 ROS 版本/换广播器实现时不会因为 QoS 不匹配而"一个消息都收不到"。

### 4.2 轨迹转换

**问题**：`QuinticTrajectory` 是连续的，需要离散成 `trajectory_msgs/JointTrajectory`。

**对策**：按 `point_dt`（默认 0.01 s = 100 Hz，与 `controller_manager.update_rate` 对齐）采样。

```cpp
const double duration = traj.duration();                       // 先校验 valid() 且 duration > 0
const std::size_t num_points = std::ceil(duration / point_dt_) + 1;   // 末点正好落在 t = duration
for (std::size_t k = 0; k < num_points; ++k) {
  const double t = std::min(k * point_dt_, duration);
  traj.sample(t, q, qdot, qddot);
  // 按**名字映射**填消息（顺序无关）：
  point.positions[i]  = q(chain_index_of_controller_joint_[i]);
  point.velocities[i] = qdot(chain_index_of_controller_joint_[i]);
  point.time_from_start = rclcpp::Duration::from_seconds(t);
}
```

三条必须写下来的约束：

1. `time_from_start` 必须**严格递增**（JTC 会拒），上面的取法天然满足；
2. 采样点数设上限（20000），否则"duration 给个巨大值"会把消息撑爆；
3. `header.stamp` **故意留 0**：JTC 对"非零且已过期"的起止时间报
   `OLD_HEADER_TIMESTAMP(-3)`；留 0 表示"从现在开始"，与 `ros2 action send_goal`
   手工测试（README §7 已验证）走的是同一条路径。

**原计划示例代码里的坑**：它写了 `point.positions.resize(n)` / `positions[i] = q(i)`
（`n` 未定义、且直接按链序填），和它自己 §4.3 声称的"按关节名映射"自相矛盾。

### 4.3 关节顺序

**问题**：`TaskRouter` 输出的轨迹关节顺序 = 链的顺序；`arm_controller` 期望的顺序 =
`controllers.yaml` 里定义的顺序（消息本身不携带"顺序的语义"，只能约定）。

**对策**：只按**名字**对齐（`chain_index_of_controller_joint_`），启动时就校验
`joint_names` 正好是链上可动关节的一个**排列**（少了 → 有关节没人执行；多了 →
JTC 报 `INVALID_JOINTS(-2)`），不合格直接启动失败，而不是等到下发轨迹才报错。

**URDF 从哪来**（原计划缺失的一环）：`loadContextFromUrdf()` 要的是**文件路径**，
而运行时的描述在 `/robot_description` 话题上（xacro 展开后的字符串）。本方案用安装好的
**纯模型** `share/kdl_tools/model/robotic_arm.urdf`（`ament_index_cpp` 定位，可用
`urdf_file` 参数覆盖）：它只描述运动学/惯量/行程，没有 `<ros2_control>` 标签，正是
`TaskRouter` 需要的全部信息，也免去在节点里再解析一次 xacro。

### 4.4 关节限位必须自己补（原计划缺失）

`loadContextFromUrdf()` **只填** 行程 `q_min/q_max` 与力矩上限 `max_torque`；
速度/加速度/jerk 上限它明确说明"由调用者显式设置"（见 `kdl_task_router.hpp` 的注释）。
而 `duration <= 0` 的自动定时**完全依赖** `max_velocity`，不填就只会得到
"未给 duration 且未设置关节速度上限，无法自动定时"。

所以本节点用一组参数补上（默认值取自 `example/control_demo.cpp`，量级与
`<limit effort>` 核对过）：`max_velocity` / `max_acceleration` / `max_jerk` /
`max_linear_velocity` / `max_linear_acceleration` / `max_angular_velocity` /
`max_angular_acceleration`。

### 4.5 自动定时要有下限（实测踩到的坑）

底层的自动定时公式 `T >= 1.875·Δ/v_max` 只保证**速度/加速度**上限，而五次插值的
jerk 是 `~Δ/T³` 量级。实测：笛卡尔目标只内收 5 cm 时，自动定时给出 `T ≈ 0.16~0.19 s`，
重建关节轨迹的 jerk 峰值 **173.6 rad/s³ > 上限 100**，解算直接失败，报文里只有一句
"轨迹不可行"，看不出真因。

对策：桥接层加参数 `min_duration`（默认 1.0 s）——先按底层自动结果来；若**解算失败**
或**时长短于下限**，就用 `min_duration` 重算一次（纯计算，代价可以忽略）。这样
`duration <= 0` 这条路径才真的可用。

### 4.6 执行器与并发（原计划缺失）

* **多线程执行器**：service 回调要阻塞几秒等 action 结果，单线程执行器会让
  `/joint_states` 订阅与 action 回调一起停摆，表现为"service 永远不返回"。
  → `MultiThreadedExecutor(…, 2)` + service/action 用 `Reentrant` 回调组、
  `/joint_states` 用独立互斥组。
* **并发调用**：本节点一次只接一条任务，第二次请求直接回 `error_code = 7`（本期不支持
  取消/排队）—— 否则两条任务会抢同一个 position 命令接口，JTC 也只会接受其中一条。

### 4.7 错误处理与超时

| 场景 | 处理 |
|---|---|
| 未收到 `/joint_states`（或缺关节/非有限值） | `error_code = 1` |
| `task_type` 未知 / `goal_joint` 长度不对 | `error_code = 2`，说明期望值 |
| 目标越界、IK 不收敛、接近奇异、jerk 超限 | `error_code = 3`，透传底层中文 message |
| action server 不可用 | `error_code = 4`（下发前 `wait_for_action_server(2 s)`） |
| goal 被拒 | `error_code = 5` |
| 等待结果超时 | `error_code = 6` |
| 已有任务在执行 | `error_code = 7` |
| JTC 判容差超限并 ABORT | 透传 JTC 的 `error_code`（-4/-5）+ `error_string` |

**超时必须是"轨迹时长 + 余量"，而且用墙钟**：原计划写"超时 5 s"，而验收用例本身
就是 5 s 的轨迹 —— 那条用例必然超时。现在是 `duration + result_timeout_margin`（默认 5 s），
用 `std::chrono` 墙钟；仿真被 `set_pause` 暂停时也会超时，这是有意的
（宁可报超时，也不要无限期挂住调用者）。

---

## 5. 文件组织

```
src/communication/
└── srv/
    └── ControlTask.srv                     # service 定义（随 kdl_tools 生成）

src/ros2_control/
├── src/
│   └── control_bridge_node.cpp             # 新节点
├── config/
│   └── controllers.yaml                    # 已有，不修改
├── launch/
│   ├── arm_mujoco_control.launch.py               # 已有，不修改
│   └── arm_mujoco_control_with_bridge.launch.py   # 新 launch
└── ...
```

---

## 6. CMake 修改

原计划的三处问题：`${CMAKE_CURRENT_SOURCE_DIR}/../communication/...` 会指到**工作区外面**
（包根是 `kdl/`，上一级是 `cpp_projects/`）；缺 `rosidl_default_generators` 的
`find_package`；`target_link_libraries(... ${PROJECT_NAME})` 在 Jazzy 上取不到 typesupport。

```cmake
# 依赖（与既有的 find_package 放在一起）
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)      # ControlTask.srv 里的 Pose
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# 生成的接口随包走，路径相对包根（CMAKE_CURRENT_SOURCE_DIR = 包根 kdl/）
rosidl_generate_interfaces(${PROJECT_NAME}
  "src/communication/srv/ControlTask.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(control_bridge_node src/ros2_control/src/control_bridge_node.cpp)
target_link_libraries(control_bridge_node
  kdl_control_lib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  sensor_msgs::sensor_msgs
  trajectory_msgs::trajectory_msgs
  control_msgs::control_msgs
)
# 取本包生成的 typesupport 目标（官方推荐的写法）
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(control_bridge_node "${cpp_typesupport_target}")

install(TARGETS control_bridge_node RUNTIME DESTINATION lib/${PROJECT_NAME})
```

`package.xml` 里同时要加（原计划漏了）：

```xml
<depend>geometry_msgs</depend>
<depend>rclcpp_action</depend>
<depend>sensor_msgs</depend>
<depend>trajectory_msgs</depend>
<depend>control_msgs</depend>
<depend>ament_index_cpp</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## 7. Launch 文件

**方案 A（采纳）**：新建 `arm_mujoco_control_with_bridge.launch.py`，用
`IncludeLaunchDescription` **包含**现有 launch，再追加桥接节点：

```python
base_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("kdl_tools"), "launch", "arm_mujoco_control.launch.py"])),
    launch_arguments={                       # 五个参数同名同默认值，直接透传
        "mujoco_model": LaunchConfiguration("mujoco_model"),
        "headless": LaunchConfiguration("headless"),
        "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
        "use_position_pid": LaunchConfiguration("use_position_pid"),
        "rviz": LaunchConfiguration("rviz"),
    }.items(),
)
bridge_node = Node(package="kdl_tools", executable="control_bridge_node",
                   name="control_bridge", output="both",
                   parameters=[{"use_sim_time": True}])
```

* 比"把桥接节点塞进现有文件"更好的地方：现有 launch 是"仿真 + 硬件 + 控制器"的最小
  可复现链路（README §9 的实测口径就是它），不动它。
* 比"复制一份现有 launch 再改"更好的地方：`headless/rviz/...` 这些参数只有一处定义。
* 桥接节点**不需要等控制器起来**再启动：它是在收到 service 请求时才
  `wait_for_action_server`，早启动反而能更早暴露 URDF/关节映射的配置错误。

---

## 8. 验收标准

1. **启动**：`ros2 launch kdl_tools arm_mujoco_control_with_bridge.launch.py headless:=true`
2. **关节空间任务**：`{task_type: 0, goal_joint: [0.5,-0.4,0.6,0.3,0.2,0.5], duration: 5.0}`
   → MuJoCo 执行 → `error_code = 0 / success = true`
3. **笛卡尔空间任务**：`{task_type: 1, goal_pose: {...}, duration: 0.0}` → 执行到位
4. **错误处理**：关节数不对 → 2；未知 `task_type` → 2；目标越界 → 3
5. **关节状态**：节点能正确从 `/joint_states` 取到当前状态（顺序无关）

实测结果见 §11。

---

## 9. 风险与对策

| 风险 | 对策 |
|---|---|
| 关节顺序不匹配 | 按名字映射，启动时校验 `joint_names` 是链上关节的一个排列 |
| 轨迹执行超时 | 超时 = 轨迹时长 + `result_timeout_margin`，墙钟计时 |
| 当前状态不准 / 未就绪 | `state_ready_` 标记；缺关节或非有限值一律判为不可用 |
| 自动定时给出过短时长，jerk 爆限 | `min_duration`（默认 1 s）下限重算（§4.5） |
| service 回调阻塞导致"/joint_states 停更" | 多线程执行器 + 回调组分离（§4.6） |
| 两条任务并发抢命令接口 | 第二次请求直接回 `error_code = 7` |
| Service 定义编译失败 | CMake 里补 `rosidl_default_generators`；package.xml 里加 `member_of_group` |
| Action client 连接失败 | 下发前 `wait_for_action_server(2 s)`，失败回 4 |
| 仿真暂停时 service 挂住 | 墙钟超时（会超时而不是永久挂住），语义写在 §4.7 |

---

## 10. 后续扩展（P5 之后）

- **多段任务**：service 请求里加 `geometry_msgs/Pose[] waypoints`（底层 `TaskRequest::cartesian_waypoints` 已支持）
- **力矩前馈**：service 请求里加 `bool compute_torque_feedforward`（底层已支持，结果在 `ControlResult::torque_feedforward`）
- **任务取消**：用 Action 代替 Service，支持取消
- **状态反馈**：用 Action 代替 Service，反馈执行进度（`/dynamic_joint_states` 已带每关节参考值）

---

## 11. 实现记录（与 §1–§10 计划的偏差 + 实测）

本节是"实现后补记"，与 `src/control/PLAN.md` 的附录 C 同一体例。

### 11.1 对计划做的修正（都是实测/源码核出来的）

| # | 计划里的问题 | 修正 |
|---|---|---|
| 1 | CMake 里 `../communication/srv/...` 指到工作区外 | 改为相对包根的 `src/communication/srv/ControlTask.srv` |
| 2 | 缺 rosidl 生成依赖与链接方式 | 补 `find_package(rosidl_default_generators)` + package.xml 三件套 + `rosidl_get_typesupport_target` |
| 3 | `rclcpp::action::Client` 不存在 | `rclcpp_action::Client` + `rclcpp_action::create_client` |
| 4 | 取链关节名用 `getSegment(i)`, `i<n_joints` | 遍历段、跳过 `KDL::Joint::None`（固定段会错位） |
| 5 | 没写 URDF 从哪来 | 用安装好的纯模型 `share/kdl_tools/model/robotic_arm.urdf`（`ament_index_cpp` 定位，可参数覆盖） |
| 6 | 没写 `ctx.joint_limits` 必须自己填 | 新增 `max_velocity/max_acceleration/max_jerk` 等参数（自动定时依赖它） |
| 7 | 超时写死 5 s（与 5 s 验收用例冲突） | `duration + result_timeout_margin`（默认 5 s），墙钟 |
| 8 | 响应没有执行结果 | 加 `error_code`（0 / 1..7 / JTC 负值三段语义） |
| 9 | 没有线程模型说明 | `MultiThreadedExecutor(2)` + Reentrant 回调组；service 与 `/joint_states` 分属不同组 |
| 10 | 自动定时在小位移下 jerk 爆限 | 新增 `min_duration`（默认 1 s）下限重算 |
| 11 | 没有并发策略 | `task_mutex_` 拒绝并发（`error_code = 7`） |
| 12 | QoS 未提 | 订阅用 `SensorDataQoS`（与 JSB 的 `SystemDefaultsQoS` 兼容） |
| 13 | launch "方案 A" 未说怎么复用 | `IncludeLaunchDescription` 包含现有 launch + 透传 5 个参数 |

### 11.2 实测（`headless:=true`，MuJoCo 3.4.0 / ROS 2 Jazzy）

启动约 1 s 后 `arm_controller` 与 `joint_state_broadcaster` 都是 `active`，
`/control_task` 可见；`control_bridge` 的启动横幅：

```
桥接就绪：service=/control_task，action=/arm_controller/follow_joint_trajectory
  URDF      : .../share/kdl_tools/model/robotic_arm.urdf
  轨迹采样  : point_dt=0.0100 s（消息里 100 个点/秒）
  自动定时  : duration<=0 时由底层估算，但不短于 min_duration=1.000 s
  关节映射  : joint1(链#0), joint2(链#1), joint3(链#2), joint4(链#3), joint5(链#4), joint6(链#5)
```

| 用例 | 请求 | 结果 |
|---|---|---|
| 1 关节空间，固定时长 | `{0: [0.5,-0.4,0.6,0.3,0.2,0.5], duration: 5.0}` | `error_code=0`，501 个路点，日志 `Goal reached, success!` |
| 2 关节空间，自动定时 | `{0: [0.2,-0.3,0.4,0.1,0.1,0.2], duration: 0.0}` | `error_code=0`，`trajectory_duration=1.0`（下限生效；底层估算是 0.574 s） |
| 3 笛卡尔空间，自动定时 | `{1: 内收 5 cm，姿态不变，duration: 0.0}` | `error_code=0`，`trajectory_duration=1.0`（**下限生效前这里失败**：jerk 173.6 > 100） |
| 3' 笛卡尔空间，固定时长 | 同上，`duration: 2.0` | `error_code=0` |
| 4 关节数不对 | `goal_joint` 长度 2 | `error_code=2`，`goal_joint 长度 2 与关节数 6 不匹配` |
| 5 未知任务类型 | `task_type: 7` | `error_code=2`，`未知 task_type = 7（0 = JOINT_SPACE，1 = CARTESIAN_SPACE）` |
| 6 目标越界 | `joint3 = 3.8` | `error_code=3`，`第 3 个关节的目标 3.8 rad 超出行程 [-2.8, 2.8] rad` |

**跟踪精度**（用例 1，跟踪与 `README.md` §9 手工 `ros2 action send_goal` 的口径一致）：

* 目标 `[0.5, -0.4, 0.6, 0.3, 0.2, 0.5]`
* 稳态实际 `[0.50000, -0.40305, 0.60287, 0.30269, 0.19886, 0.50000]`
* 最大误差 **0.00305 rad ≈ 0.175°**（远小于 `controllers.yaml` 里 0.05/0.08 rad 的 `goal` 容差）

即：**同一套控制参数下，走桥接下发的轨迹与手工 `send_goal` 的结果一致** —— 桥接层
只是"翻译 + 转发"，没有引入额外误差。
