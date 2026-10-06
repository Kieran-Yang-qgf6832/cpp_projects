# MuJoCo + ros2_control 机械臂控制方案（力矩控制版）

把 **MuJoCo 当作 ros2_control 的一块"硬件"**：`MujocoSystemInterface` 插件在
`read()/write()` 里与 MuJoCo 交换数据，上层控制器完全不用知道下面跑的是仿真还是真机。

本目录是这条链路的所有"胶水"：机器人描述、控制器配置、启动文件、可视化配置。
二进制源码（插件本体）在 `src/mujoco_ros2_control/`，机械模型在 `src/model/`。

> **本目录已从「位置轨迹 + JTC」改为「力矩控制 + 自研控制器」。**
> 现在的链路是：
> ```
> 调用者 ──srv /control_task──▶ kdl_control_node（任务解算 + 离散 + 逆动力学前馈）
>                              ──topic /control_reference──▶ kdl_effort_controller（前馈 τ_ff + PID）
>                                                          ──effort 接口──▶ MuJoCo <motor>
> ```
> 设计/决策/实现记录 → [`PLAN_TORQUE.md`](./PLAN_TORQUE.md)；
> 实现与联调中 5 个问题的完整排查 → [`力矩控制问题报告.md`](./力矩控制问题报告.md)。

---

## 1. 目录与职责

| 路径 | 作用 | 会被谁读 |
|---|---|---|
| `urdf/robotic_arm_mujoco.urdf.xacro` | 机器人描述：include 模型 + `<ros2_control>` 硬件/关节配置（命令接口 = `effort`） | `robot_state_publisher`、`ros2_control_node` |
| `config/controllers.yaml` | `controller_manager`（`update_rate: 500`）与控制器参数 | `ros2_control_node`、`spawner` |
| `config/mujoco_pids.yaml` | **PID 增益来源**（`pid_gains.position.<joint>`） | `kdl_effort_controller` |
| `src/kdl_effort_controller.cpp` / `include/kdl_effort_controller.hpp` | 力矩控制器插件本体（控制环内） | 编译成 `libkdl_effort_controller.so`，由 CM 以插件加载 |
| `controller_plugins.xml` | 插件注册（`kdl_tools/KdlEffortController`） | `pluginlib` |
| `launch/arm_mujoco_control.launch.py` | 仿真 + 硬件 + 控制器 | 人 / CI |
| `launch/arm_mujoco_control_with_bridge.launch.py` | 上面的链路 + `kdl_control_node`（见 §10） | 人 / CI |
| `rviz/arm_mujoco.rviz` | 看 TF 与机器人模型 | `rviz2`（`rviz:=true`） |
| `../control/src/node/kdl_control_node.cpp` | 任务解算节点：`/control_task` → 轨迹 → `/control_reference` | 编译成 `kdl_control_node` |
| `../communication/srv/ControlTask.srv` | `/control_task` 服务定义 | `kdl_control_node`、调用者 |
| `../communication/msg/ControlStatus.msg` | 控制器 → 解算节点的状态回报 | 同上 |
| `../model/robotic_arm.urdf` | **纯模型**（SolidWorks 导出），运动学/惯量唯一真源 | xacro include、KDL 建链 |
| `../model/robotic_arm_torque.xml` / `scene_torque.xml` | **力矩版 MJCF**（`<motor>`）—— 当前默认 | MuJoCo |
| `../model/robotic_arm.xml` / `scene.xml` | 位置伺服版 MJCF（`<position>`）—— 保留可回退 | MuJoCo（换 `mujoco_model` 时） |

> 模型文件刻意保持"导出工具原始形态"，控制相关配置全部放在本目录，
> 这样机械设计改版重新导出时不会覆盖掉控制配置。

---

## 2. 系统架构

```
                    ┌──────────────────────── URDF/xacro 描述 ────────────────────────┐
                    │  <ros2_control name="arm_hw" type="system">                     │
                    │    <hardware> mujoco_ros2_control/MujocoSystemInterface + 参数   │
                    │    <joint name="jointN"> command: effort / state: pos,vel,effort │
                    └─────────────────────────────────────────────────────────────────┘
   robot_state_publisher ──/robot_description(transient_local)──┐
          │                                                      │
          ├──/tf,/tf_static──> RViz                              ▼
          └──/joint_states<──┐                    ┌──────────────────────────────┐
                             │                    │  controller_manager (RT 线程) │
   MuJoCo 物理线程 (mj_step) │                    │  read() → update() → write()  │
          │                  │                    └───────┬──────────────▲───────┘
          │                  │       state_interface 回读 │              │ command_interface 下发(effort)
          │                  └──── joint_state_broadcaster ┘              │
          │                                              ┌──────────────┴───────────────┐
          │                                              │ kdl_effort_controller         │
          │                                              │ τ = τ_ff + kpΔq + kdΔq̇ + ki∫Δq│
          │                                              │ 参考 ← /control_reference      │
          │                                              └──────────────────────────────┘
          ▼
   ┌──────────────────────────────────────────────────────────────────────┐
   │ MujocoSystemInterface（插件，即"硬件驱动"）                            │
   │   · mj_model / mj_data 由插件持有，内部起物理线程 + 渲染线程            │
   │   · read()  : 取最近一次 mj_step 后的 qpos/qvel/qfrc_actuator → 关节状态 │
   │   · write() : effort 命令 → actuator 的 ctrl，暂存后由物理线程在下一次   │
   │               mj_step 前取走（不是直接写 mj_data，见 §5）              │
   │   · 每个 mj_step 后发布 /clock（use_sim_time 的依据）                  │
   └──────────────────────────────────────────────────────────────────────┘
```

要点：**插件自己拥有并驱动 MuJoCo**（物理线程 + 渲染线程），ros2_control 只是它的
一个"外部接口"。所以不存在"外部步进仿真"的说法，控制环与物理环是**解耦的两个线程**，
靠共享缓冲区 + `/clock` 对齐。

---

## 3. 关节 / 执行器 / 接口配置

三处必须对齐，改一处就要看另外两处：

| ros2_control 关节 | MJCF 关节 | MJCF 执行器 | 命令接口 | 状态接口 |
|---|---|---|---|---|
| `joint1` | `joint1` | `<motor name="motor1" gear="1" ctrlrange="-50 50">` | **effort** | position/velocity/effort |
| `joint2` | `joint2` | `motor2`（±50） | effort | 同上 |
| `joint3` | `joint3` | `motor3`（±50） | effort | 同上 |
| `joint4` | `joint4` | `motor4`（±50） | effort | 同上 |

* **关节**：`src/model/robotic_arm.urdf` 里的 `joint1..joint4`（revolute，行程见 `<limit>`；
  四个关节的 `<limit effort>` 均为 50 N·m）。
* **执行器**：MJCF 的 `<actuator>` 用 **`<motor>` 纯力矩**，`ctrl` 的语义就是关节力矩 [N·m]
  （`gear=1`，插件不做缩放），与 ros2_control 的 `effort` 命令接口天然对应。
* ⚠️ **`<position>` 与 `<motor>` 是编译期二选一、不能运行期切换**：effort 命令接口对
  `<position>`/`<velocity>` 执行器会**直接报错并跳过**（关节收不到任何力矩）。
  位置伺服版是 `robotic_arm.xml` + `scene.xml`，仍保留可回退。
* **控制律与增益不在插件里，而在 `kdl_effort_controller` 里**：
  前馈 `τ_ff = inverseDynamics(chain, q_ref, 0, 0)`（空闲/结束时）或轨迹点自带的 `effort`
  （执行中），反馈 `τ = τ_ff + kp·Δq + kd·Δq̇ + ki·∫Δq`，再经 `u_clamp`/`i_clamp`
  （来自 `mujoco_pids.yaml`）与 `±URDF effort` 双层限幅。
* **传动**：本臂是 1:1 直连，**不需要 `<transmission>`**。插件按关节名先在 MJCF 里找同名
  joint、再找驱动它的 actuator，找到就直接把关节命令写到该 actuator 的 `ctrl`。
  只有"一个关节由多个执行器驱动"或"非 1:1 减速比"才需要
  `transmission_interface/SimpleTransmission`，且要注意两个坑（写在 xacro 末尾注释里）：
  1. `<actuator name="...">` 填的是 **MJCF 里被驱动关节的名字**，不是 MJCF 执行器名；
  2. 若该关节名能直连到执行器，直连优先、减速比会被覆盖。

### 状态与命令接口来源（插件侧）

| 接口 | 数据来源 | 说明 |
|---|---|---|
| `position` | `mj_data->qpos[jnt_qposadr]` | 关节角/位移 |
| `velocity` | `mj_data->qvel[jnt_dofadr]` | 关节速度 |
| `effort`（state） | `mj_data->qfrc_actuator[jnt_dofadr]` | 执行器施加的广义力（≈力矩） |
| `effort`（command） | 写入 `mj_data->ctrl[actuator_id]` | 对 `<motor>` 执行器即力矩 |

本链路的话题/服务（实测）：

| 名称 | 类型 | 用途 |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 每个物理步发布一次，仿真时间基准 |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` 汇总的状态（50 Hz） |
| `/mujoco_actuators_states` | `sensor_msgs/JointState` | 插件直接发布的执行器状态（调试用，含 effort） |
| `/control_task` | `kdl_tools/srv/ControlTask` | **任务下发入口**（由 `kdl_control_node` 提供） |
| `/control_reference` | `trajectory_msgs/JointTrajectory` | 控制点：position/velocity/effort（由解算节点发出） |
| `/kdl_effort_controller/status` | `kdl_tools/msg/ControlStatus` | 控制器执行状态（active/progress/error_code） |
| `/mujoco_ros2_control_node/set_pause` | `mujoco_ros2_control_msgs/SetPause` | 暂停/继续 |
| `/mujoco_ros2_control_node/step_simulation` | `.../StepSimulation` | 单步（`steps` 步） |
| `/mujoco_ros2_control_node/reset_world` | `.../ResetWorld` | 复位到初始状态/关键帧 |

---

## 4. 启动流程

```bash
# 0) 环境（RMW 是 CycloneDDS，必须先 source，否则找不到 libddsc）
source /opt/ros/jazzy/setup.bash
cd /home/kangy/MyProjects/cpp_projects/kdl
colcon build --paths .          # 安装 model/ 与 src/ros2_control/ 到 share/kdl_tools
source install/setup.bash

# 1) 仿真 + 控制器（不含解算节点）
ros2 launch kdl_tools arm_mujoco_control.launch.py                 # 带 MuJoCo 窗口
ros2 launch kdl_tools arm_mujoco_control.launch.py headless:=true  # 无界面
ros2 launch kdl_tools arm_mujoco_control.launch.py rviz:=true      # 额外开 RViz

# 2) 仿真 + 控制器 + 解算节点（要发任务就用这个）
ros2 launch kdl_tools arm_mujoco_control_with_bridge.launch.py headless:=true
```

launch 内部顺序（不要随意打乱）：

1. `xacro` 展开 `urdf/robotic_arm_mujoco.urdf.xacro` → `robot_description`；
2. `robot_state_publisher`：既广播 TF，又把 URDF 以 `transient_local` 发到
   `/robot_description` —— **`ros2_control_node` 就是从这个话题取 URDF 的**；
3. `mujoco_ros2_control/ros2_control_node`（改造版 CM）：加载 `arm_hw` 硬件、
   构造 MuJoCo 模型、起物理线程与渲染线程，然后等 `/clock` 跑起来；
4. `spawner` 依次加载并激活 `joint_state_broadcaster`、`kdl_effort_controller`
   （`--controller-manager-timeout 60`，因为首次编译 MJCF+STL、等时钟可能超过默认 10 s）；
5. （仅 `with_bridge`）`kdl_control_node`。

启动成功的标志（实测日志）：

```
[controller_manager]: Using ROS clock for triggering controller manager cycles.
[controller_manager]: Received robot description from topic.
[MujocoSystemInterface]: Loading 'mujoco_model' from: '.../share/kdl_tools/model/scene_torque.xml'
[MujocoSystemInterface]: Using MuJoCo motor or custom actuator for the joint : 'joint1'   (×4)
[controller_manager]: Loading controller : 'kdl_effort_controller' of type 'kdl_tools/KdlEffortController'
[kdl_effort_controller]: 已配置：4 个关节，参考话题 /control_reference，增益来自 .../config/mujoco_pids.yaml
[kdl_effort_controller]:   joint1: p=80.290 i=0.000 d=10.710 u=[-50.000,50.000] i_clamp=[-1.000,1.000]
[kdl_effort_controller]:   joint2: p=18.620 i=0.000 d=2.483  u=[-50.000,50.000] i_clamp=[-1.000,1.000]
[kdl_effort_controller]:   joint3: p=1.900  i=0.000 d=0.078  u=[-50.000,50.000] i_clamp=[-1.000,1.000]
[kdl_effort_controller]:   joint4: p=0.355  i=0.000 d=0.047  u=[-50.000,50.000] i_clamp=[-1.000,1.000]
[MujocoSystemInterface]: Joint joint1: effort control enabled (position, velocity disabled)   (×4)
[kdl_effort_controller]: 已激活：进入重力锁位
```

> ⚠️ **已知现象（问题 3）**：控制器被 spawner 激活之前（约 0.5 s），`effort` 命令一直是 0，
> 机械臂会在重力下**自由下落**，最后锁在"摔到哪算哪"的位形。这是当前未根治的结构性问题，
> 详见 [`力矩控制问题报告.md`](./力矩控制问题报告.md) §3。

---

## 5. MuJoCo 与 ros2_control 的同步 / 通信机制

**两条独立线程 + 两级缓冲**，没有"锁死"关系：

* **物理线程**：`while(run) { apply_staged_control_inputs(); mj_step(); publish_control_state(); publish_clock(); }`
  —— 按墙钟（×`sim_speed_factor`）追赶，把多步 `mj_step` 攒在一批里跑，每步 2 ms（实测
  `/clock` ≈ **500.0 Hz**，即 `<option timestep="0.002">`）。
* **控制线程**（`controller_manager`，`SCHED_FIFO` 优先级 50）：
  `read() → update() → write()`，周期 = `1/update_rate` = 2 ms。
* **状态回读**：物理线程每步把 `qpos/qvel/qfrc_actuator/sensordata` 复制进一个小的
  `control_state_`（加锁），`read()` 只做一次浅拷贝 —— 所以 `read()` 拿到的是"最近一个已完成
  物理步"的状态，不是阻塞等来的。
* **命令下发**：`write()` 算出 `ctrl[]` 后交给 `apply_control_data()` 暂存
  （`ctrl_staged_`），物理线程在**下一次 `mj_step` 之前**取走。即命令是"写进下一个控制
  周期生效的零阶保持"，不是立刻改 `mj_data`。
* **时间对齐**：`use_sim_time=true` 时，CM 不是按墙钟睡，而是按 **`/clock`（仿真时钟）**
  睡：`sleep_until(current_time + 1/update_rate)`。所以控制环的节拍是**仿真时间**的，
  与物理步严格对齐，且不受渲染卡顿/墙钟抖动影响 → 结果可复现。

```
仿真时间轴（每格 2 ms = 1 次 mj_step）
|----|----|----|----|----|----|----|----|----|----|-->  t_sim
^    ^    ^    ^    ^    ^    ^    ^    ^    ^    ^
read/update/write   （500 Hz，每 2 ms 一次，恰好每步一次）
```

> 力矩环取 500 Hz（= 物理步速率）是为了把离散化相位滞后压到最小。
> 若降到 100 Hz（每 5 步一次），`kp` 给大了就更容易在采样间振荡（见问题报告 §2）。

---

## 6. 仿真步长与控制周期匹配

**黄金规则：`1 / (update_rate × timestep)` 必须是正整数。**

| 项目 | 位置 | 当前值 |
|---|---|---|
| 物理步长 | `src/model/scene_torque.xml` 的 `<option timestep>`（显式 0.002） | 2 ms（500 Hz） |
| 控制周期 | `config/controllers.yaml` 的 `controller_manager.update_rate` | **500 Hz（2 ms）** |
| 每控制周期物理步数 | 上两者之比 | **1** |

* 为什么必须整数比：`/clock` 只在**物理步边界**跳变（每 2 ms 一次），CM 的
  `sleep_until` 只能被"跨过目标时刻"的那个 tick 唤醒。若比例不是整数（例如
  `update_rate=300` → 每周期 1.667 步），唤醒时刻会被量化到下一个步边界，实际控制周期
  变成 4 ms（≈250 Hz）而不是 3.33 ms —— 表现为"控制比标称慢、日志没有报错但跟踪变差"。
* `sim_speed_factor`（xacro 参数）只改**仿真时间相对墙钟**的倍率，**不改变**
  仿真步长与控制周期的比例。
* 各控制器还可以给自己的 `update_rate`（见 `joint_state_broadcaster: 50`），
  但它必须是 `controller_manager.update_rate` 的**整数分频**（500/10 = 50）。
* **CM 日志会提示 `Enforcing command limits is disabled`** —— URDF `<limit>` 不会被用来夹取
  命令，真正生效的是 MJCF 的 `<motor ctrlrange>`（力矩）与 joint 的 `actuatorfrcrange`。

---

## 7. 调试流程

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 1) 控制器 / 硬件接口状态
ros2 control list_controllers
#   joint_state_broadcaster  ...  active
#   kdl_effort_controller    kdl_tools/KdlEffortController  active
ros2 control list_hardware_interfaces          # 看 effort 命令接口是否 claimed
ros2 control list_hardware_components

# 2) 时钟与物理步频率
ros2 topic hz /clock                           # 期望 ≈ 500
ros2 topic echo /clock --once

# 3) 状态回读
ros2 topic echo /joint_states --field position --once
ros2 topic echo /mujoco_actuators_states sensor_msgs/msg/JointState

# 4) 控制器状态
ros2 topic echo /kdl_effort_controller/status --once

# 5) 发一条任务（需要 with_bridge launch）
ros2 service call /control_task kdl_tools/srv/ControlTask \
  "{task_type: 0, goal_joint: [1.0,-1.0,1.2,0.8], duration: 0.0}"

# 6) 暂停 / 单步 / 复位（研究控制细节时很好用）
ros2 service call /mujoco_ros2_control_node/set_pause \
  mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"
ros2 service call /mujoco_ros2_control_node/step_simulation \
  mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 10}"
ros2 service call /mujoco_ros2_control_node/reset_world \
  mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: ''}"
```

常见故障对照表：

| 现象 | 原因 | 处理 |
|---|---|---|
| `Effort command interface ... not supported with position or velocity actuator. Skipping it.` | 用了位置版 MJCF（`<position>`） | 力矩模式必须用 `scene_torque.xml`（`<motor>`） |
| 启动后机械臂不是从零位开始，而是"摔"到一个位形 | 控制器激活前 ~0.5 s 无控制自由下落（问题 3） | 见问题报告 §3 |
| 机械臂大幅摆动 / 力矩长期饱和 | PID 增益与 `u_clamp` 不匹配（线性区太窄） | 见问题报告 §2（按 `M(q)` 反推增益） |
| 某个关节怎么都到不了目标（时间跑完却差一截） | 多半是连杆自碰撞卡住 | 返回 `error_code=8`；见问题报告 §4 |
| `Plugin ... MujocoSystem` 加载失败 | `<plugin>` 名写错 | 必须是 `mujoco_ros2_control/MujocoSystemInterface` |
| `Failed loading controller kdl_effort_controller` | 插件没装/名字不对 | 确认 `share/kdl_tools/controller_plugins.xml` 存在，`type` 为 `kdl_tools/KdlEffortController` |
| `MuJoCo model file '...' does not exist!` | `mujoco_model` 没被 xacro 展开或路径不存在 | 用 xacro 生成描述；默认指向 `share/kdl_tools/model/scene_torque.xml` |
| CM 一直卡住、控制器 spawner 超时 | `/clock` 没发布（仿真没跑起来） | 看插件日志是否 `Running in HEADLESS mode` / 模型是否加载成功 |
| 模型发散、`Diverged` 警告 | 增益过大（尤其小惯量腕关节），或 `timestep` 太大 | 按 `M(q)` 重算增益；或降 `timestep` |
| 手臂在零位附近自己抖动/啃在一起 | 连杆凸包接触力 | 已在 `robotic_arm_torque.xml` 的 `<contact><exclude>` 里排除相邻**与非相邻**对 |

---

## 8. 实时性说明

* 控制线程会尝试 `SCHED_FIFO` / 锁内存，普通用户下必然看到
  `Could not enable FIFO RT scheduling policy: Operation not permitted` —— **仿真场景下可忽略**。
* 仿真里"实时性"的实际含义是"控制周期不要超时"：CM 检测到超周期会打
  `Overrun detected! ... missed cycles`。本项目 `update_rate=500` + 模型很小，
  实测**没有出现超周期**；若机器较慢可降到 250/100（记得保持整数分频，见 §6）。
* 真正的抖动来源是 MuJoCo 渲染线程（非 headless 时开 GLFW 窗口）。若关心时序一致性，
  用 `headless:=true`。
* 想跑得比墙钟快用 `sim_speed_factor`（例如 `5.0`），对控制环的**仿真时间周期**没有影响。

---

## 9. 实测结论

`headless:=true`：

* `/clock` ≈ **500 Hz** → 物理步 2 ms；`/joint_states` = **50 Hz**（JSB 10 分频）；
* 起完约 0.55 s 内 `effort` 全为 0（无控制自由下落，问题 3），之后控制器接管并锁位；
* 端到端任务（`with_bridge`）：

| 用例 | 结果 |
|---|---|
| 关节空间 `[1.0,-1.0,1.2,0.8]`（起 `[0.87,2.40,0.30,0.96]`），`duration: 0` → 自动 6.50 s | `error_code=0`；稳态 `[1.0000,-1.0000,1.2000,0.8000]`，**残差 0.0000 rad**，零过冲 |
| 关节空间 `[0.0,0.8,0.0,1.4]`，`duration: 0` → 自动 3.44 s | `error_code=0`；稳态 `[0.0000,0.8000,0.0000,1.4000]`，**残差 0.0000 rad** |
| 关节空间 `[1.2,-1.2,1.4,0.6]`，`duration: 0` → 自动 3.83 s | `error_code=0`；稳态 `[1.2000,-1.2000,1.4000,0.6000]`，**残差 0.0000 rad**，零过冲 |

> 过程中遇到的 5 个问题（教学库重力符号、PID 不可用、自由下落、假自碰撞、完成判据）
> 的现象与排查见 [`力矩控制问题报告.md`](./力矩控制问题报告.md)。

---

## 10. 任务层接入：`kdl_control_node` + `kdl_effort_controller`

§1–§9 描述的是"仿真 + 力矩控制器"。要让它**自己解算任务并执行**，再加两个组件：

```
调用者 → /control_task(service) → kdl_control_node
                                    → TaskRouter::dispatch()（纯计算：越界/IK/限位全在这里判）
                                    → ControlResult::joint_trajectory（连续五次多项式）
                                    → 按 point_dt 离散 + 逐点 inverseDynamics → 控制点
                                    → /control_reference(topic, JointTrajectory)
                                    → kdl_effort_controller（前馈 + PID）→ MuJoCo
                                    ← /kdl_effort_controller/status（active 跳变）
```

**为什么是"topic + 控制器"，而不是"节点直接发 action 给 JTC"**：力矩必须在与读状态、
写命令**同一个控制周期内**算出来。作为 CM 插件，`update()` 天然满足；独立节点绕
`/joint_states`（50 Hz）做反馈会引入大延迟。

### 10.1 启动与调用

```bash
ros2 launch kdl_tools arm_mujoco_control_with_bridge.launch.py headless:=true

# 关节空间：目标关节角 + 8 s
ros2 service call /control_task kdl_tools/srv/ControlTask \
  "{task_type: 0, goal_joint: [1.0, -1.0, 1.2, 0.8], duration: 0.0}"

# 笛卡尔空间：目标末端位姿（相对 base_link），duration <= 0 表示自动定时
ros2 service call /control_task kdl_tools/srv/ControlTask \
  "{task_type: 1, duration: 0.0, goal_pose: {position: {x: 0.54, y: 0.14, z: 1.06},
    orientation: {x: 0.943, y: -0.019, z: -0.332, w: 0.011}}}"

ros2 interface show kdl_tools/srv/ControlTask     # 看完整字段与错误码说明
```

成功时的响应：

```
kdl_tools.srv.ControlTask_Response(error_code=0, success=True,
  message='执行完成：轨迹 8.000000 s', trajectory_duration=8.0)
```

### 10.2 `error_code` 怎么读

| 值 | 含义 | 先看哪里 |
|---|---|---|
| 0 | 成功（轨迹跑完**且**终点残差在 `tracking_tolerance` 内） | — |
| 1 | 收不到完整 `/joint_states` | `joint_state_broadcaster` 是否 active；QoS（节点用 `SensorDataQoS`） |
| 2 | 请求本身不合法（`task_type` / `goal_joint` 长度） | `message` 里已写明期望值 |
| 3 | 解算失败：越界 / IK 不收敛 / 接近奇异 / jerk 超限 | `message` 是底层中文原因 |
| 4 | 参数/配置错误 | 启动日志 |
| 5 | 控制器未接手轨迹 | `ros2 control list_controllers`；关节名是否与 `kdl_effort_controller.joints` 一致 |
| 6 | 等执行结果超时（轨迹时长 + 余量，墙钟） | 仿真是否被 `set_pause` 暂停了 |
| 7 | 已有任务在执行（不支持排队/取消） | 等上一条返回 |
| 8 | 轨迹时间跑完但**终点未到位** | 多半是自碰撞卡住或力矩限幅；`message` 里有具体关节与残差 |

### 10.3 决定"能不能一次跑通"的参数

`kdl_control_node`（节点私有参数）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `point_dt` | 0.002 s | 轨迹离散成控制点的间距，与 `update_rate`（500 Hz）对齐 |
| `max_trajectory_points` | 50000 | 点数上限（DDS 单条消息限制保护） |
| `min_duration` | 1.0 s | 自动定时（`duration <= 0`）的下限：底层只保证速度/加速度上限，而 jerk ~ Δ/T³，位移小的时候会给出过短的 T 导致不可行。设 0 = 完全听底层 |
| `tracking_tolerance` | 0.05 rad | **终点到位自检阈值**：控制器"跑完时间"后，节点等机械臂停稳再核对残差，超阈值返回 `error_code=8` |
| `result_timeout_margin` | 5.0 s | 等执行结果的余量（墙钟） |
| `max_velocity` 等 | `1.0 / [30,30,30,30] / 100` | `duration <= 0` 时自动定时的依据 |

`kdl_effort_controller`（控制器参数）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `joints` | `joint1..joint4` | 关节顺序（必须与消息 `joint_names` 是同一集合） |
| `reference_topic` | `/control_reference` | 参考轨迹话题 |
| `urdf_file` | 空 = `share/kdl_tools/model/robotic_arm.urdf` | 建 KDL 链做重力前馈 |
| `pid_config_file` | 空 = `share/kdl_tools/config/mujoco_pids.yaml` | PID 增益来源 |
| `feedforward` | `true` | 是否使用轨迹点里的 `effort` 作为前馈（关闭则执行期只有 PD，空闲仍有重力补偿） |
| `status_publish_rate` | 100 Hz | `~/status` 发布频率 |

`joint_names`（默认 `joint1..joint4`）决定 `JointTrajectory` 消息里的顺序，必须与
`controllers.yaml` 的 `kdl_effort_controller.joints` 一致；解算节点只按**名字**把链的关节映射到
消息里，映射不成立（少关节/多关节/名字对不上）会**直接启动失败**并打印链上的关节名。

### 10.4 两个实现上的硬约束

* **`kdl_control_node` 必须多线程执行器**：service 回调会阻塞着等 `/status`（几秒），
  单线程执行器会让 `/joint_states` 与 `/status` 回调一起停摆 —— 表现为"service 永远不返回"。
* **一次只接一条任务**：第二条并发请求直接回 `error_code = 7`（两条任务会抢同一批
  effort 命令接口，控制器也只会执行其中一条）。

### 10.5 实测（`headless:=true`，本机）

| 用例 | 结果 |
|---|---|
| 关节空间 `[1.0,-1.0,1.2,0.8]`（起 `[0.87,2.40,0.30,0.96]`），`duration: 0` → 自动 6.50 s | `error_code=0`；四轴终点残差 **0.0000 rad**，零过冲 |
| 关节空间 `[0.0,0.8,0.0,1.4]`，`duration: 0` → 自动 3.44 s | `error_code=0`；四轴终点残差 **0.0000 rad**，静止后速度 5e−14 rad/s |
| 负向：关节数给错 / 未知 `task_type` / 目标越界 | `error_code` 分别是 2 / 2 / 3 |
| 负向：`duration` 给得过短（速度峰值超 `max_velocity`） | `error_code=3`，`message` 给出具体关节与峰值速度 |
| 终点未到位（`tracking_tolerance` 超阈值） | `error_code=8`，`message` 给出具体关节与残差 |

> ⚠️ **已知残留问题**：大行程轨迹在**峰值速度段**会出现 0.1~0.2 s 的高频振荡，实测关节速度冲到
> 14~22 rad/s（URDF 上限 10 rad/s），随后自恢复且不影响最终精度。7 次实测中出现 5 次。
> 详见 [`力矩控制问题报告.md`](./力矩控制问题报告.md) 文末注记。

更细的设计与决策见 [`PLAN_TORQUE.md`](./PLAN_TORQUE.md)；问题排查见
[`力矩控制问题报告.md`](./力矩控制问题报告.md)。
