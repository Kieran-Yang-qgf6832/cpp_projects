# 力矩控制改造实现计划（PLAN_TORQUE）

> 需求来源：在既有「位置轨迹（JTC）」链路上，改为「前馈 + 反馈的力矩控制」，并把任务解算层
> （`src/control`）上移为独立节点，直接托管 service。
>
> 本文件是该需求的落地设计：决策记录、架构、接口、控制器控制律、仿真侧改动、构建接线、
> 分阶段验收与风险。**实现时以本文件的接口与规格为准；若要改签名或控制律，先改本文件。**
>
> 状态：设计已冻结（8 项关键决策见 §1），尚未实现。

---

## 0. 需求原文与对应关系

| 需求原文 | 本计划的落实位置 |
|---|---|
| control 接受上层 srv，路由到具体 task，解算轨迹 | §5 `kdl_control_node` |
| 按控制周期 dt 离散轨迹，控制点含 position/velocity/effort | §5.2、§4.1 |
| control_bridge 读控制点、算 effort、写入 mujoco 命令接口 | §6 `kdl_effort_controller` |
| 控制点含 position/velocity/effort；kp/kd/ki 取自 `mujoco_pids.yaml` | §4.1、§6.2 |
| `effort = torque_ref + kp·Δpos + kd·Δvel + ki·∫Δpos` | §6.1 控制律 |
| mujoco 使用力矩模式，返回 position/velocity/effort | §7 MJCF / 状态接口 |

> 与需求的两处**有意偏离**（经确认）：
> 1. 原需求要求用**共享内存**传递控制点 → 改为 `trajectory_msgs/JointTrajectory` 话题。理由见 §1、§3.3。
> 2. 原需求把 `control_bridge` 当作独立节点 → 改为 **ros2_control 控制器插件**（必须跑在控制环内）。理由见 §1 F1。

---

## 1. 决策记录（已冻结，勿轻易改动）

| # | 决策 | 备选与理由 |
|---|---|---|
| D1 | `control_bridge` = **ros2_control 控制器插件**（`kdl_effort_controller`） | 力矩必须闭环在 CM 的 `update()` 内。独立节点/环外定时器会因 `/joint_states`(50 Hz)+DDS 延迟而抖动甚至发散 |
| D2 | 控制点传输 = **`trajectory_msgs/JointTrajectory` 话题** | 该消息天然带 `position/velocity/effort/time_from_start`，语义与「控制点」一一对应；共享内存对「一次性批量」是过度设计，且引入锁/撕裂风险 |
| D3 | 空闲/轨迹结束 = **PD 锁位 + 控制器内重力前馈** `τ_ff=G(q)` | 纯 PD 会有稳态下沉 `Δq=G/kp`（joint2 约 2°）；yaml 的 `ki` 全为 0，无法消差。重力前馈可实现零下沉，代价是控制器引入 KDL 依赖 |
| D4 | service 语义 = **阻塞到执行完**（保留旧 UX） | 控制器用 `~/status` 话题回报 `active`；节点等 `active:true→false`。不引入 action，改动最小 |
| D5 | MJCF = **新增 torque 版文件**（`robotic_arm_torque.xml`+`scene_torque.xml`） | `<motor>` 与 `<position>` 是编译期二选一，不可运行时切换；新增文件可保留 position 链路作回退 |
| D6 | CM `update_rate = 500 Hz`（= 物理步 2 ms） | 力矩环相位裕度最大；`kp=300` 级别增益在 500 Hz 下稳定裕度充足。代价是 CPU 略高 |
| D7 | srv 归属 = **`kdl_control_node`**（归属 `src/control`） | 只在「可执行层」破 `src/control` 不依赖 ROS 消息的边界；`kdl_control_lib` 仍保持纯 KDL |
| D8 | 抢占策略 = **单飞行**（忙则 `error_code=7`） | 沿用旧语义，避免中途替换参考导致跳变 |

---

## 2. 目标与边界

**目标**：把「任务目标」翻译成一条关节空间轨迹，再用**前馈 + 反馈**的方式以力矩驱动 MuJoCo，实现比位置伺服更"贴近真实机器人"的控制结构。

**链路**：

```
调用者 ──srv /control_task──▶ kdl_control_node
                              │ ① 取 /joint_states 的 q_now
                              │ ② TaskRouter::dispatch()  → 关节五次轨迹（C⁴）
                              │ ③ 按 point_dt 采样 (q,q̇,q̈) → inverseDynamics → τ_ff
                              │ ④ 离散成 JointTrajectory(position/velocity/effort)
                              │ ⑤ publish /control_reference
                              │ ⑥ 等 ~/status 的 active:false → 回 response
                              ▼
                        topic /control_reference
                              ▼
                       kdl_effort_controller（CM 500 Hz update）
                              │ 参考索引推进 → 读 state(q,q̇) → 算 τ → 写 effort 接口
                              ▼
                       MujocoSystemInterface: control_data->ctrl[id] = τ  →  MuJoCo <motor>
                              └── /joint_states、~/status 回报
```

**三条硬边界**：

1. **不改底层教学库**（`Tools`/`Kinematics`/`Interpolation`/`Dynamics`）的公开接口与实现；只调用。
2. **`kdl_control_lib` 保持纯 KDL**（不新增 ROS 消息依赖）；ROS 依赖只出现在新节点可执行文件里。
3. **控制律参数与限幅必须有单一出处**：增益取 `mujoco_pids.yaml`；力矩上限取 URDF `<limit effort>`。

**明确不做**：轨迹时间最优、奇异规避的在线重规划、力控/阻抗、多段任务的 service 入参扩展（`TaskRequest` 已预留）。

---

## 3. 关键前置事实（实现前必须知道，均有源码依据）

| # | 事实 | 依据 |
|---|---|---|
| F1 | effort 命令接口**只对 `<motor>`/`<general>` 执行器有效**；对 position/velocity 会 `ERROR` 并 skip | `docs/hardware_interface.rst` 兼容表；`mujoco_system_interface.cpp:1644-1662` |
| F2 | 改造前 MJCF 全为 `<position>`（旧 6 轴版 kp=400..3000，dampratio=1.0） | `src/model/robotic_arm.xml`（该文件现已换成 4 轴模型，kp=100/800/500/50） |
| F3 | `write()` 把 effort 命令**原样写 `ctrl`，无 gear 缩放** | `mujoco_system_interface.cpp:1100-1103` |
| F4 | `mujoco_pids.yaml` **在 effort 模式下不会被插件使用**（只在 position/velocity 接口 + motor 执行器时喂 `control_toolbox::PidROS`） | `mujoco_system_interface.cpp:1284-1325, 1589` |
| F5 | control_toolbox 的 PID 公式 = `p·e+i·∫e+d·ė` + `u_clamp`/`i_clamp`，与需求公式同语义 | 同上 |
| F6 | **yaml 里 position 段所有关节 `i=0.0`**（积分项恒为 0） | `config/mujoco_pids.yaml` 的 `pid_gains.position` 段 |
| F7 | URDF 与 MJCF 的关节轴/顺序**逐条一致** → 前馈力矩无符号/顺序问题 | `robotic_arm.urdf` vs `robotic_arm.xml` |
| F8 | 状态接口已有 `effort`，实测值 = `qfrc_actuator` | `mujoco_system_interface.cpp:926`；xacro `:80` |
| F9 | 反馈率不匹配：CM `100` vs JSB `50`（新方案 CM=500） | `controllers.yaml:19,41` |
| F10 | router 已有前馈实现，但本方案改为节点内**逐采样点**精确计算（网格对齐更可靠） | `kdl_task_router.cpp:220-299` |

---

## 4. 接口设计

### 4.1 控制点（复用 `trajectory_msgs/JointTrajectory`）

- 话题：**`/control_reference`**，`trajectory_msgs/msg/JointTrajectory`，QoS `reliable` + `volatile` + `keep_last(1)`。
- `joint_names` = 控制器关节顺序（joint1..joint4）；控制器按**名字**对齐。
- `points[k].positions/velocities/efforts`：关节参考 `q_ref/q̇_ref/τ_ff`，单位 rad、rad/s、N·m。
- `points[k].time_from_start`：**严格递增**，首点 0，末点 = `duration`。
- `header.stamp` 留 0（与旧实现一致，语义 = 「从现在开始」）。
- `header.frame_id` 置空。

> **采样网格 `point_dt`（默认 0.002 s = 500 Hz）与 CM 控制周期解耦**：控制器按时间做**线性插值**，
> 即使消息网格比控制周期稀也不会失配。默认对齐 500 Hz 是为了让控制器直接索引、零插值误差。

### 4.2 Service（沿用 `kdl_tools/srv/ControlTask`）

不改字段。语义变化仅一处：

- `trajectory_duration`：从「执行完成的轨迹时长」变为「已下发的规划时长」；
- `error_code`：`0` 成功；`1..7` 沿用旧分段（1 无状态 / 2 请求非法 / 3 解算失败 / 4 控制器不可用 / 5 控制器未接手 / 6 执行超时 / 7 忙）；`<0` 预留控制器/硬件错误。

### 4.3 状态回报（新增 `kdl_tools/msg/ControlStatus.msg`）

```
std_msgs/Header header
bool    active          # 是否正在执行
int32   error_code      # 0 正常；>0 控制器自身错误
string  message
float64 progress        # 0..1（执行进度）
float64 duration        # 当前参考的总时长 [s]
```

- 控制器经 **realtime publisher**（`try_publish`）以 ~100 Hz 发到 **`~/status`**（相对控制器命名空间）。
- 节点订阅 `~/status`（与控制器同一命名空间），实现 §5.3 的等待状态机。

---

## 5. `kdl_control_node`（`src/control/src/node/kdl_control_node.cpp`）

### 5.1 装配（构造期）

1. 参数：`urdf_file`（默认 `share/kdl_tools/model/robotic_arm.urdf`）、`point_dt`（默认 0.002）、
   `min_duration`（默认 1.0）、`max_trajectory_points`（默认 50000）、`result_timeout_margin`（默认 5.0）、
   速度/加速度/jerk/笛卡尔限位（沿用旧桥参数名与默认值）。
2. `TaskRouter::loadContextFromUrdf` 装配上下文，并**显式补上** `joint_limits` 与 `cartesian_limits`（同旧桥）。
3. 建立「控制器关节名 ↔ 链关节下标」映射，校验 `joint_names` 是链上可动关节的一个排列。
4. 订阅 `/joint_states`（`SensorDataQoS`）、订阅 `~/status`、advertise `/control_reference`、创建 `/control_task` service。
5. 执行器：**多线程**（service 回调要阻塞等 status，不能让 `/joint_states` 停摆）。

### 5.2 一次调用的流程

```
1. 取关节状态快照 q_now/qdot_now（未就绪 → error_code=1）
2. srv 请求 → TaskRequest（沿用旧桥 buildTaskRequest；未识别类型 → 2）
3. result = router_.dispatch(task, q_now, qdot_now)
   - 自动定时（duration<=0）沿用旧的 min_duration 下限重算
   - 失败 → error_code=3
4. 离散：
   n = ceil(duration/point_dt)+1（超 max_trajectory_points → 3）
   for k: t=min(k*point_dt, duration)
          traj.sample(t, q, qdot, qddot)       # 连续轨迹 → 采样
          τ_ff = kdl_dynamics::inverseDynamics(chain, q, qdot, qddot)
          填 position/velocity/effort（按 §5.4 名字映射）
5. publish /control_reference
6. 等 status（§5.3）→ 回 response
```

> 为什么**不用** `TaskRouter::compute_torque_feedforward`：它的采样网格是 `sample_dt` 且有点数上限，
> 与本节点的 `point_dt` 网格并不保证对齐。逐点直接调用 `inverseDynamics` 简单、精确、无耦合。

### 5.3 等待执行的返回状态机

```
发送轨迹
  A 等 status.active == true    超时 2 s     → 失败 5（控制器没接手：没激活？joint 名不匹配？）
  B 等 status.active == false   超时 duration+result_timeout_margin
                                              → 失败 6
  回 response.error_code = status.error_code（0 → success=true）
```

- 单飞行：`task_mutex_` `try_lock` 失败 → `error_code=7`。

### 5.4 采样与名字映射

- `traj.sample` 给出的 `q/qdot/qddot` 是**链顺序**；消息里按**控制器顺序**填，用
  `chain_index_of_controller_joint_[i]`（同旧桥）。
- `point.positions/velocities/efforts` 长度 = 控制器关节数。

---

## 6. `kdl_effort_controller`（ros2_control 控制器插件）

文件：`src/ros2_control/include/kdl_effort_controller.hpp`、`src/ros2_control/src/kdl_effort_controller.cpp`。

### 6.1 控制律（RT 线程内，禁止分配/解析/日志刷屏）

```
Δq = q_ref − q_meas          Δq̇ = q̇_ref − q̇_meas        # 直接用量测速度，不做数值微分
I += Δq · period
τ_pid = kp·Δq + kd·Δq̇ + ki·I
τ_pid ← 限幅：ki·I ∈ [i_clamp_min, i_clamp_max]；τ_pid ∈ [u_clamp_min, u_clamp_max]
τ = clamp(τ_ff + τ_pid, ±URDF_effort)                     # effort 来自 URDF <limit effort>
写 effort 接口
```

- **`τ_ff` 二选一**：
  - 执行中：参考点 `effort` 字段（节点算好的逆动力学 τ）；
  - 空闲/结束保持：`kdl_dynamics::gravityTorque(chain, q_meas)`（消重力 → 零稳态下沉）。
- **限幅双层**：`u_clamp/i_clamp` 取自 yaml（PID 语义）；`±URDF_effort` 取自 URDF（物理上限）。
  MuJoCo `<motor ctrllimited>` 的夹取是最后一道，不作为唯一防线。

### 6.2 装配（`on_configure`，非 RT）

1. 声明/读取参数：`joints`、`command_interfaces`、`state_interfaces`、`urdf_file`、`pid_config_file`、
   `feedforward`（默认 true）、`status_publish_rate`（默认 100）。
2. `kdl_tools::buildChainFromUrdfFile(urdf_file)` → `KDL::Chain`（重力前馈用）。
3. `yaml-cpp` 解析 `pid_config_file`，取
   `/** → ros__parameters → pid_gains → position → <joint> → {p,i,d,u_clamp_min/max,i_clamp_min/max}`；
   逐关节装入固定数组。缺项 → `on_configure` 返回 ERROR（早失败）。
4. 订阅 `/control_reference`（回调在非 RT 线程，解析成预分配数组），经
   `realtime_tools::RealtimeBuffer` 交接给 update。
5. 创建 `~/status` 的 realtime publisher（`realtime_tools::RealtimePublisher`）。

### 6.3 状态机（`update()` 内）

```
on_activate: hold_ref := 当前 q；active := false；I := 0；elapsed := 0

update(time, period):
  if RealtimeBuffer 有新轨迹:
      q_ref_seq := 新轨迹；elapsed := 0；I := 0；active := true；error_code := 0
  if active:
      elapsed += period
      if elapsed >= duration:
          active := false；hold_ref := 末点；          # 锁末点
      else:
          (q_ref, v_ref, τ_ff) := 按 elapsed 线性插值
  if !active:
      q_ref := hold_ref；v_ref := 0；τ_ff := gravityTorque(chain, q_meas)
  按 §6.1 算 τ → 写 effort 接口
  按 status_publish_rate 发布 ~/status
```

- **激活即锁位**：`on_activate` 把 `hold_ref` 设为当前 q 并施加重力前馈 → 控制器一激活臂就不会下坠。
- 执行中 `τ_ff` 来自轨迹；结束后切到 `G(q)`。因末点 `q̇=q̈=0`，逆动力学在末点 = `G(q_ref)`，切换连续，无跳变。
- `I` 在新轨迹与激活时清零。

### 6.4 命名与插件注册

`controller_plugins.xml`：

```xml
<library path="kdl_effort_controller">
  <class name="kdl_tools/KdlEffortController"
         type="kdl_tools::KdlEffortController"
         base_class_type="controller_interface::ControllerInterface">
    <description>前馈 τ + PID 反馈的关节力矩控制器（MuJoCo &lt;motor&gt;）。</description>
  </class>
</library>
```

---

## 7. 仿真侧改动

### 7.1 MJCF（新增 torque 版）

`src/model/robotic_arm_torque.xml`：与 `robotic_arm.xml` 完全一致，仅 `<actuator>` 段改为
（`ctrlrange` 取 URDF effort；本模型四个关节均为 50）：

```xml
<actuator>
  <motor name="motor1" joint="joint1" gear="1" ctrllimited="true" ctrlrange="-50 50" />
  <motor name="motor2" joint="joint2" gear="1" ctrllimited="true" ctrlrange="-50 50" />
  <motor name="motor3" joint="joint3" gear="1" ctrllimited="true" ctrlrange="-50 50" />
  <motor name="motor4" joint="joint4" gear="1" ctrllimited="true" ctrlrange="-50 50" />
</actuator>
```

`src/model/scene_torque.xml`：复制 `scene.xml`，`<include file="robotic_arm_torque.xml"/>`。

### 7.2 xacro

`src/ros2_control/urdf/robotic_arm_mujoco.urdf.xacro`：

- `<command_interface name="position"/>` → `<command_interface name="effort"/>`；
- 去掉 `pids_config_file` 的 `xacro:if` 块（effort 模式下插件不用它，改由控制器参数传入）；
- `mujoco_model` 默认值由 launch 指向 `scene_torque.xml`（arg 已存在，不改名）。

### 7.3 controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500                      # D6
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    kdl_effort_controller:
      type: kdl_tools/KdlEffortController

joint_state_broadcaster:
  ros__parameters:
    update_rate: 50

kdl_effort_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    urdf_file: ""                         # 空 = 用 share/kdl_tools/model/robotic_arm.urdf
    pid_config_file: ""                   # 空 = 用 share/kdl_tools/config/mujoco_pids.yaml
    feedforward: true
    status_publish_rate: 100.0
```

> `arm_controller`(JTC) 与 `arm_forward_position_controller` 从 controllers.yaml 移除（position 接口已不存在）。

### 7.4 launch

- `arm_mujoco_control.launch.py`：`mujoco_model` 默认指向 `model/scene_torque.xml`；spawner 列表改为
  `["joint_state_broadcaster", "kdl_effort_controller"]`；`robot_description` 由 xacro 展开（effort 接口）。
- `arm_mujoco_control_with_bridge.launch.py`：改为再追加 `kdl_control_node`（名字建议 `control_node`），
  透传参数保持与基线 launch 一致。

---

## 8. 文件组织

```
src/communication/
├── srv/ControlTask.srv                 # 不变
└── msg/ControlStatus.msg               # 新增

src/control/
└── src/node/kdl_control_node.cpp       # 新增（ROS 只出现在这一层）

src/ros2_control/
├── include/kdl_effort_controller.hpp   # 新增
├── src/kdl_effort_controller.cpp       # 新增
├── controller_plugins.xml              # 新增
├── src/control_bridge_node.cpp         # 退休（删除 target/install）
├── config/controllers.yaml             # 修改
├── config/mujoco_pids.yaml             # 只读复用（增益来源）
├── urdf/robotic_arm_mujoco.urdf.xacro  # 修改
└── launch/*.launch.py                  # 修改

src/model/
├── robotic_arm_torque.xml              # 新增
└── scene_torque.xml                    # 新增
```

---

## 9. 构建与安装（根 `CMakeLists.txt`）

```cmake
# ---- 新增依赖 ----
find_package(std_msgs REQUIRED)                 # ControlStatus.msg 的 Header
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- 接口：srv + msg 一起生成 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "src/communication/srv/ControlTask.srv"
  "src/communication/msg/ControlStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# ---- 力矩控制器插件（真正的控制环内 bridge）----
add_library(kdl_effort_controller SHARED
  src/ros2_control/src/kdl_effort_controller.cpp
)
target_include_directories(kdl_effort_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_control/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)
target_link_libraries(kdl_effort_controller
  controller_interface::controller_interface
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  trajectory_msgs::trajectory_msgs
  kdl_tools_lib kdl_dynamics_lib        # 建链 + 重力前馈
  yaml-cpp::yaml-cpp
)
pluginlib_export_plugin_description_file(
  controller_interface src/ros2_control/controller_plugins.xml)

# ---- 任务解算节点（接 srv，发 /control_reference）----
add_executable(kdl_control_node
  src/control/src/node/kdl_control_node.cpp
)
target_link_libraries(kdl_control_node
  kdl_control_lib
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  sensor_msgs::sensor_msgs
  trajectory_msgs::trajectory_msgs
  kdl_dynamics_lib                       # inverseDynamics → τ_ff
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(kdl_control_node "${cpp_typesupport_target}")

install(TARGETS kdl_effort_controller kdl_control_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
```

`package.xml` 追加：`std_msgs`、`controller_interface`、`pluginlib`、`realtime_tools`、`hardware_interface`、
`yaml_cpp_vendor`（或 `yaml-cpp`）；移除不再需要的 `rclcpp_action`、`control_msgs`（JTC action client 已退休，
若其他文件仍用则保留）。

**退休**：删除 `src/ros2_control/src/control_bridge_node.cpp` 对应的 `add_executable` / `target_link_libraries` /
`install` 三处。

---

## 10. 分阶段实施与验收

每个阶段结束都必须 `cd kdl && colcon build --paths .` 通过，且该阶段验收可复现。

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| **P1** | torque MJCF + xacro(effort) + controllers/launch 切 500 Hz | 起仿真后：手动给零力矩 → 臂因重力下垂；给常值 τ → 对抗重力停住；`ros2 control list_hardware_interfaces` 见 `joint*/effort` 命令接口 |
| **P2** | `kdl_control_node`：srv + 解算 + 2 ms 离散 + 前馈 + publish | `ros2 topic echo /control_reference`：点数 = `ceil(dur/0.002)+1`、`time_from_start` 严格递增、`efforts` 非全零（约等于重力项） |
| **P3a** | `kdl_effort_controller`：仅 IDLE_HOLD + 重力前馈 | 激活控制器后臂**不再下坠**，静止时 `Δq≈0`（零下沉） |
| **P3b** | 加 PD（`τ_ff` 暂用 0） | 给一个静态参考，臂到位且无稳态下沉（重力前馈生效）、不发散 |
| **P3c** | 执行态改用消息 `τ_ff` | 跟踪一条关节轨迹，末端进入容差；无持续饱和 |
| **P4** | `~/status` + 节点等待状态机 + 单飞行 | 端到端 service 返回 `error_code=0`；执行中再调返回 7；故意填错 `goal_joint` 长度返回 2 |
| **P5** | 调参 + 记录 q/τ 曲线 + 文档同步 | 跟踪误差 < 容差、无持续饱和；`PLAN.md`/`README.md`/`启动指南.md` 更新 |

**P3 顺序不可颠倒**：先证明「能悬停」，再证明「能跟随」；否则一旦发散，分不清是重力前馈还是 PD 的问题。

---

## 11. 风险与对策

| 风险 | 等级 | 对策 |
|---|---|---|
| 力矩环抖动/发散（环外或周期失配） | 🔴→已消除 | 控制器插件在 500 Hz 环内；参考按周期索引 |
| 空闲/结束力矩为 0 → 臂下坠 | 🔴→已消除 | IDLE_HOLD + 重力前馈；激活即锁位 |
| PID 限幅/积分饱和 | 🔴 | `u_clamp`/`i_clamp`（yaml）+ `±URDF_effort` 双层限幅；`I` 在激活/新轨迹时清零 |
| MJCF 不可逆切换 | 🟡→已缓解 | 新增 torque 文件，position 链路保留 |
| 100 Hz 稳定性不足 | 🟡→已缓解 | 提到 500 Hz；`kp=300` 时 `ωn·T≈0.39 rad`，裕度充足 |
| 增益仍需实测整定 | 🟡 | P3 分级验证；必要时下调 `kp` 或 PM 阶段再调 |
| 起始瞬态（生成 q_now 到执行有时间差） | 🟡 | 轨迹首点 = 生成时 q_now；控制器已锁位，误差有限；必要时对 τ 加斜率限幅 |
| 模型差异（惯量数值、`<dynamics>` 阻尼/摩擦） | 🟡→**已发生** | 换 4 轴臂后旧 PID 增益直接导致 joint3 自激（实测 ±50 rad/s）。已按各关节等效惯量重算并实测通过，推导见 `config/mujoco_pids.yaml` 头部注释 |
| 消息过大 / 点数爆表（24 s@2 ms ≈ 2501 点/5 s） | 🟡 | `point_dt` 与网格解耦（控制器线性插值），可上调 `point_dt`；`max_trajectory_points` 上限保护 |
| 控制器 `~/status` 与节点命名空间不一致 | 🟢 | 统一用 `kdl_effort_controller/status`；launch 校验 |
| `defaultGravity()` 与 MJCF `<option gravity>` 不一致 | 🟢 | 实现时核对，不一致则显式传 `(0,0,-9.81)` |
| CM 500 Hz 与物理步非整数分频 | 🟢 | 确认 MJCF timestep = 0.002（默认值）；必要时在 `<option>` 显式写死 |

---

## 12. 与既有实现的偏差、假设与待办

### 12.1 相对旧实现（P5 位置桥）的偏差

| 项 | 旧实现 | 本方案 |
|---|---|---|
| 执行接口 | `control_msgs/FollowJointTrajectory` action → JTC | `trajectory_msgs/JointTrajectory` topic → 自研力矩控制器 |
| 命令接口 | position | effort |
| 反馈 | 无（JTC 内部用 position 接口） | 控制器读 position/velocity state interface |
| 结果回报 | JTC 的 action result | 控制器 `~/status` topic |
| 文件 | `src/ros2_control/src/control_bridge_node.cpp` | 退休；节点移到 `src/control/src/node/`，控制器新增 |

### 12.2 前馈来源

- 采用**节点内逐采样点 `inverseDynamics`**，不改 `TaskRouter` 的 `compute_torque_feedforward`（其网格与
  上限不适合本用途）。`TaskRequest::compute_torque_feedforward` 保持默认 `false`。

### 12.3 待办（实现时确认，已列入 §11 的 🟢 项）

1. `kdl_dynamics::defaultGravity()` 与 MJCF `gravity="0 0 -9.81"` 一致性。
2. `robotic_arm.xml` 是否显式声明 `<option timestep>`（当前依赖 MuJoCo 默认 0.002）。
3. 500 Hz 下 `kp=300` 的实测稳定性（如不够，降 `kp` 或再提 `update_rate`）。
4. `arm_controller`/`arm_forward_position_controller` 移除后，README 中位置链路的手工验证章节需要标注为「历史」。

---

## 13. 实现记录（与 §1–§12 的偏差）

> 本节是"实现后补记"。**完整的问题现象、排查过程、解决方案与效果见
> [`力矩控制问题报告.md`](./力矩控制问题报告.md)**；这里只列与计划的接口/规格差异。

| 项 | 计划（§） | 实际实现 | 原因（详见问题报告） |
|---|---|---|---|
| 空闲重力前馈 | `gravityTorque(chain, q_meas)`（§6.1） | `inverseDynamics(chain, q_ref, 0, 0)` | ① 在**保持参考点**算才能零静差（setpoint gravity comp）；② 与执行态 τ_ff 同函数、符号天然一致 |
| 增益 | 直接取 `mujoco_pids.yaml` 原值（§6.2） | 按实测 `M(q)` 用 `kp=Mωn², kd=2ζMωn`（ωn=15, ζ=1）重调；换 4 轴臂后又按 `τ_g/kp` 柔度修正了小惯量关节 | 原值配 ±50 clamp 全程饱和 → bang-bang（问题二）；换臂后 joint3 柔度达 1.76 rad → 终点残差 1.3 rad |
| MJCF 关节 | 仅 `<motor>` 替换（§7.1） | 旧 6 轴臂额外加了关节阻尼；**4 轴臂未加**（新 URDF 无 `<dynamics>`，四个关节全零阻尼） | 控制器激活前有 ~1.4 s 无控制窗口，需压住自由下落速度（问题三） |
| MJCF 接触 | 未提及 | 非相邻连杆两两 `<exclude>`（4 轴模型共 10 对，已覆盖全部连杆组合） | 网格凸包近似造成假自碰撞，把关节顶在半路（问题四；旧 6 轴臂实测为 link3↔link5） |
| 完成判据 | 参考时间跑完即 `active=false`（§6.3） | 节点在终点**等停稳**后核对残差，超阈值返回 `error_code=8`（新增 `tracking_tolerance`，默认 0.05 rad） | "跑完时间" ≠ "到位"（问题五） |
| `ControlTask.srv` 错误码 | 0 / 1..7 / `<0`（§4.2） | 0 / 1..8 / `<0`（新增 8 = 未到位） | 同上 |
| `kdl_dynamics` / `dynamics_demo` | — | 修正 `gravityTorque` 的重力补偿符号注释与用例 | 记号写反会加倍重力（问题一） |

**尚未根治**：问题三（激活前自由下落）。旧 6 轴臂曾用关节阻尼缓解；4 轴模型未加阻尼，且 joint2 行程为 ±3.1（旧臂 ±1.57），下落幅度更大。

---

## 附录 A：本计划用到的接口索引

**`kdl_control`**

```cpp
TaskRouter::loadContextFromUrdf(urdf_file, ctx, message);
ControlResult TaskRouter::dispatch(req, q_now, qdot_now) const;
// ControlResult.joint_trajectory : QuinticTrajectory
```

**`kdl_interpolation`**

```cpp
bool QuinticTrajectory::sample(double t, JntArray &q, JntArray &qdot, JntArray &qddot) const;
double QuinticTrajectory::duration() const;
```

**`kdl_dynamics`**

```cpp
IdResult    inverseDynamics(const KDL::Chain&, const JntArray &q, const JntArray &qdot,
                            const JntArray &qddot, const KDL::Vector &gravity = defaultGravity());
GravityResult gravityTorque(const KDL::Chain&, const JntArray &q,
                            const KDL::Vector &gravity = defaultGravity());
```

**`kdl_tools`**

```cpp
bool buildChainFromUrdfFile(const std::string &urdf_file, KDL::Chain &chain,
                            const std::string &base_link = kDefaultBaseLink,
                            const std::string &tip_link  = kDefaultTipLink);
```

**ros2_control**

```cpp
// 控制器生命周期
controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&);
controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);
controller_interface::InterfaceConfiguration command_interface_configuration() const;
controller_interface::InterfaceConfiguration state_interface_configuration() const;
controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&);
// 实时交接
realtime_tools::RealtimeBuffer<T>;
realtime_tools::RealtimePublisher<T>;
```

**MuJoCo 侧（本仓库 `src/mujoco_ros2_control`）**

```cpp
// write(): control_data->ctrl[actuator.mj_actuator_id] = actuator.effort_interface.command_;
// read() : actuator.effort_interface.state_ = control_state_.qfrc_actuator[actuator.mj_vel_adr];
// 兼容性：effort 命令接口仅支持 MuJoCo motor/general 执行器（position/velocity 执行器会 ERROR+skip）
```
