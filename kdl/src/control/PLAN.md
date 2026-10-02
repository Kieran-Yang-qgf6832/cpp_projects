# `src/control` 实现计划

> 需求来源：本目录的 `plan` 文件。
> 本文件是该需求的落地设计：目录划分、对外接口、两个任务的解算流程、构建/安装接线、
> 分阶段验收标准与风险。实现时以本文件的接口签名为准；若要改签名，先改本文件。

---

## 0. 需求原文与对应关系

| `plan` 里的要求 | 本计划的落实位置 |
|---|---|
| 头文件放 `include/`，`.cpp` 放 `src/` | §2 目录结构 |
| 调用 Dynamics、Interpolation、Kinematics 的底层算法完成解算 | §4、§5、§6（不修改底层模块，只调用） |
| 文件组织：`router`（任务基类 + 按任务信号调度） | §3.2、§3.3 |
| 文件组织：`tasks`（具体任务解算） | §4、§5 |
| 任务1：关节空间 当前关节状态 → 目标关节状态 | §4 |
| 任务2：笛卡尔空间 当前末端位姿 → 目标末端位姿（不一定走直线） | §5 |
| 目前只做两个任务，其他后续实现 | §3.3（未注册类型返回"未实现"）、§10 扩展点 |

---

## 1. 目标与边界

**目标**：把"任务请求 + 机器人当前状态"翻译成一串对底层模块的调用，产出一条**可直接执行**的
关节空间轨迹。

**三条硬边界**：

1. **只调用、不修改**：`kdl_tools` / `kdl_kinematics` / `kdl_interpolation` / `kdl_dynamics`
   的任何头文件与实现都不动。control 只能用它们的公开接口拼流程。
2. **统一出口是 `kdl_interpolation::QuinticTrajectory`**。所有任务的解算结果都落到关节空间轨迹：
   `ros2_control` 只能下发关节量，笛卡尔任务也必须经 IK 落到关节侧。上层因此只需要认识一种结果。
3. **不抛异常**：失败用"结果结构 + 中文 message"表达（与 `IkResult` / `TrajectoryResult` /
   `IdResult` 完全一致），因为调用方可能处在实时控制循环里。

**明确不做的事**（本期）：轨迹时间最优规划、奇异规避、力控/阻抗、多机协同、在线重规划。
其中"奇异预警"只预留接口位置（§10）。

---

## 2. 目录与文件

```
src/control/
├── PLAN.md                                   # 本文件
├── include/
│   ├── router/
│   │   ├── kdl_task_base.hpp                 # 任务基类 + 任务信号/结果/上下文
│   │   ├── kdl_task_router.hpp               # 任务调度路由
│   │   └── kdl_control_print.hpp             # 打印（可选文件，见 §3.5）
│   └── tasks/
│       ├── kdl_joint_space_task.hpp          # 任务1
│       └── kdl_cartesian_space_task.hpp      # 任务2
└── src/
    ├── router/
    │   ├── kdl_task_base.cpp
    │   ├── kdl_task_router.cpp
    │   └── kdl_control_print.cpp
    └── tasks/
        ├── kdl_joint_space_task.cpp
        └── kdl_cartesian_space_task.cpp
```

约定（与现有模块一致）：

* 命名空间 `kdl_control`；文件名统一 `kdl_` 前缀（对应 `kdl_fk.hpp`、`kdl_quintic.hpp`）。
* `src/control/include` 是包含根，引用方式：`#include "router/kdl_task_router.hpp"`、
  `#include "tasks/kdl_joint_space_task.hpp"`；跨模块引用直接 `#include "kdl_ik.hpp"`。
* `include/` 与 `src/` 的子目录结构镜像（`router/`、`tasks/`），安装时子目录保持原样。
* 头文件用 `#ifndef KDL_CONTROL__XXX_HPP_` 卫哨，与现有风格一致。
* 求解逻辑不关心打印（各模块都有独立 print 头文件，此处沿用）。

---

## 3. 对外接口设计

### 3.1 任务信号与结果（`router/kdl_task_base.hpp`）

```cpp
namespace kdl_control
{

/// 任务类型：路由按这个字段分派。新增任务时在这里加枚举值。
enum class TaskType
{
  kUnknown,        ///< 未识别/未实现
  kJointSpace,     ///< 任务1：关节空间运动
  kCartesianSpace  ///< 任务2：笛卡尔空间运动
};

/// 任务状态：给上层的状态机用（解算是纯函数，状态由调用方维护）
enum class TaskStatus
{
  kIdle,      ///< 未开始
  kReady,     ///< 校验通过，等待下发
  kRunning,   ///< 执行中
  kFinished,  ///< 到位
  kFailed     ///< 失败（message 说明原因）
};

/// 任务解算需要的**静态前提**：装配一次，所有任务复用。
struct RobotContext
{
  KDL::Chain chain{};                        ///< 由 URDF 截取的链
  KDL::JntArray q_min{};                     ///< 关节下限（URDF <limit lower>），长度 = 关节数
  KDL::JntArray q_max{};                     ///< 关节上限（URDF <limit upper>）
  KDL::JntArray max_torque{};                ///< 力矩上限（URDF <limit effort>），用于动力学可行性检查

  kdl_interpolation::JointLimits     joint_limits{};      ///< 速度/加速度/jerk 上限（逐关节）
  kdl_interpolation::CartesianLimits cartesian_limits{};  ///< 末端线/角速度、线/角加速度上限

  /// 上下文是否已装配完整（chain 有自由度 + 限位长度匹配）
  bool valid() const;
};

/// 任务信号：调用者填这个结构体，路由据此选任务
struct TaskRequest
{
  TaskType    type = TaskType::kUnknown;
  std::string name;                     ///< 可选，日志用

  // ---- 任务1 用 ----
  KDL::JntArray goal_joint{};           ///< 目标关节角 [rad]，长度 = 关节数

  // ---- 任务2 用 ----
  KDL::Frame              goal_pose{};                  ///< 单段目标位姿（相对基座）
  std::vector<KDL::Frame> cartesian_waypoints{};        ///< 多段/非直线时给中间路点（含目标）
  bool                    has_start_pose = false;        ///< 是否显式指定起点位姿
  KDL::Frame              start_pose{};                  ///< 显式起点；缺省时用 FK(q_now)

  // ---- 时间参数（两条任务共用）----
  double              duration  = 0.0;   ///< 单段时长 [s]；<=0 表示自动估算（见 §4/§5）
  std::vector<double> durations{};       ///< 多段时长 [s]，长度 = 段数；为空时用 duration 或自动估算
  kdl_interpolation::WaypointBehavior behavior =
      kdl_interpolation::WaypointBehavior::kStop;        ///< 仅任务2有效

  // ---- 开关 ----
  bool   compute_torque_feedforward = false;  ///< 是否调用 Dynamics 算力矩前馈
  bool   check_torque_limit        = false;   ///< 是否用 max_torque 判定可行性
  double sample_dt  = 0.01;                   ///< 任务2 笛卡尔采样步长 [s]，默认对齐 100 Hz
  double joint_jump_threshold = 0.5;          ///< 任务2 IK 解跳变阈值 [rad]
  double position_tolerance    = 1e-3;        ///< 任务2 自检位置残差阈值 [m]
  double orientation_tolerance = 8.7e-3;      ///< 任务2 自检姿态残差阈值 [rad]（≈0.5°）
};

/// 解算结果：所有任务同构
struct ControlResult
{
  bool     success = false;
  std::string message;                  ///< 失败原因（中文，可直接打印）
  TaskType type    = TaskType::kUnknown;

  kdl_interpolation::QuinticTrajectory joint_trajectory{};  ///< 统一出口（success 为真时有效）
  double duration = 0.0;                                    ///< = joint_trajectory.duration()

  std::vector<KDL::JntArray> torque_feedforward{};          ///< 可选：各采样点的 τ [N·m]
  std::vector<double>        torque_times{};                ///< 与 torque_feedforward 对应的时刻 [s]

  // 任务2 的质量指标；任务1 恒为 0
  double position_error    = 0.0;        ///< FK 自检位置残差峰值 [m]
  double orientation_error = 0.0;        ///< FK 自检姿态残差峰值 [rad]
  unsigned int ik_failures = 0;          ///< IK 收敛失败的点数（成功时为 0）
};

}  // namespace kdl_control
```

### 3.2 任务基类（`router/kdl_task_base.hpp`）

```cpp
class TaskBase
{
public:
  virtual ~TaskBase() = default;

  /// 任务类型（路由的键）
  virtual TaskType type() const = 0;

  /// 任务名（打印/日志用）
  virtual const char * name() const = 0;

  /// 前置校验：尺寸是否匹配、目标是否在限位内。不计算轨迹。
  /// @param message [out] 失败原因（中文）
  virtual bool validate(const RobotContext & ctx, const TaskRequest & req,
                        std::string & message) const = 0;

  /// 解算：由当前状态 + 任务请求生成关节空间轨迹。
  /// @note 无状态、无副作用：同一组输入必然得到同一组输出，便于对比与单测。
  virtual ControlResult solve(const RobotContext & ctx, const TaskRequest & req,
                              const KDL::JntArray & q_now,
                              const KDL::JntArray & qdot_now) const = 0;
};
```

设计要点：

* **`solve()` 是纯函数**：任务对象不持有"当前状态"。当前关节状态由调用者每个周期传进来，
  避免"任务里存了一份过期的 q"这种最典型的 bug。
* **`validate()` 与 `solve()` 分开**：上层可以在下发命令前先做一次廉价校验（不产生轨迹），
  失败就根本不进入解算。

### 3.3 任务路由（`router/kdl_task_router.hpp`）

```cpp
class TaskRouter
{
public:
  TaskRouter();                                   ///< 构造时注册两个内置任务

  /// 装配机器人上下文（链 + 关节限位 + 力矩上限）。
  void setContext(const RobotContext & ctx);
  const RobotContext & context() const;

  /// 覆盖/新增任务实现（后续任务的扩展点）
  void registerTask(std::shared_ptr<TaskBase> task);

  /// 是否已注册该类型的任务
  bool hasTask(TaskType type) const;

  /// 可读的任务名（未注册时返回 "unimplemented"）
  const char * taskName(TaskType type) const;

  /// 主入口：按 req.type 分派；统一做校验、动力学前馈与结果收尾。
  ControlResult dispatch(const TaskRequest & req,
                         const KDL::JntArray & q_now,
                         const KDL::JntArray & qdot_now) const;

  /// 便捷装配：从 URDF 文件一次性得到 RobotContext（链 + 限位 + 力矩上限 + 可用限位）
  /// @note 内部用 kdl_tools::buildChainFromUrdfFile + loadUrdfModel，
  ///       读取 <limit lower/upper/effort>，并可由调用者覆盖速度/加速度/jerk 上限。
  static bool loadContextFromUrdf(const std::string & urdf_file,
                                  RobotContext & ctx,
                                  std::string & message,
                                  const std::string & base_link = kdl_tools::kDefaultBaseLink,
                                  const std::string & tip_link  = kdl_tools::kDefaultTipLink);

private:
  /// 横切步骤：把轨迹采样后逐个路点喂给 kdl_dynamics::inverseDynamics
  /// 填 ControlResult::torque_feedforward / torque_times，并按需检查 max_torque
  bool computeTorqueFeedforward(const TaskRequest & req, ControlResult & result) const;

  RobotContext ctx_{};
  std::map<TaskType, std::shared_ptr<TaskBase>> tasks_{};
};
```

路由负责的**三件横切事情**（两个任务不必各写一遍）：

1. **分派**：`type` 未注册 → 返回 `success = false` + `"任务类型未实现（后续扩展）"`，不崩溃。
2. **统一校验**：上下文是否 `valid()`、`q_now/qdot_now` 长度是否等于 `chain.getNrOfJoints()`、
   `type` 是否与请求内容自洽（例如关节空间任务却给了 `goal_pose`）。
3. **收尾**：动力学前馈与力矩上限检查（§6）。

### 3.4 上下文装配（`loadContextFromUrdf`）

按顺序做四件事，全部失败都返回 `false` + 中文 `message`：

1. `kdl_tools::buildChainFromUrdfFile(urdf_file, chain, base_link, tip_link)` → `ctx.chain`；
2. `kdl_tools::loadUrdfModel(urdf_file, model)` → 拿 URDF 的限位；
3. 按"跳过固定段、按可动关节计数"的规则（照抄 `example/fk_ik_demo.cpp` 里 `readJointLimits()`
   的写法）填 `q_min / q_max / max_torque`；
4. `joint_limits.max_velocity / max_acceleration / max_jerk` 与 `cartesian_limits` 由参数
   文件或调用者显式设置（URDF 里没有 jerk 上限，速度上限虽有 `velocity=`，但默认只作参考，
   是否启用由调用者决定）；三项的"长度 0 = 不校验"语义由 `JointLimits` 自己保证。

### 3.5 打印（`router/kdl_control_print.hpp`，可选但建议保留）

与各模块一致，求解逻辑不关心打印。计划提供四个函数：

```cpp
void printTaskStatus(TaskStatus status, std::ostream & os = std::cout);
void printRobotContext(const RobotContext & ctx, std::ostream & os = std::cout);
void printControlResult(const ControlResult & result, std::ostream & os = std::cout);
void printTorqueFeedforward(const ControlResult & result, std::ostream & os = std::cout);
```

`printControlResult` 打印：任务名/类型、是否成功、失败 message、轨迹概览（复用
`kdl_interpolation::printTrajectorySummary`）、路点表（`printKnotTable`）、以及任务2 的
`position_error / orientation_error`。

---

## 4. 任务1：关节空间运动

`JointSpaceTask::solve(ctx, req, q_now, qdot_now)`：

| 步骤 | 动作 | 调用的底层接口 |
|---|---|---|
| 1 | 前置校验（由 `validate()` 承担）：`goal_joint` 长度 = 关节数；逐关节落于 `[q_min, q_max]`，越界报"第 i 个关节目标 x 超出 [a, b]" | — |
| 2 | 确定段时长 `T` | 见下方"时间参数" |
| 3 | 构轨迹：路点 `{q_now, goal_joint}`、时长 `{T}` | `kdl_interpolation::buildQuinticTrajectory(waypoints, durations, ctx.joint_limits)` |
| 4 | 失败：原样上抛 message | `TrajectoryResult::message` |
| 5 | 成功：填 `ControlResult`（轨迹 + `duration`） | `QuinticTrajectory::duration()` |
| 6 | 可选：动力学前馈与力矩检查由**路由**统一做 | `kdl_dynamics::inverseDynamics` |

**时间参数（第 2 步）三种来源，优先级从高到低**：

1. `req.duration > 0` → 直接用；
2. `req.durations` 非空 → 取第一个（关节空间任务只支持单段；多段属于后续扩展）；
3. 都没有 → **按速度上限自动估算**：

   ```
   T = max_i ( 1.875 * |Δq_i| / v_max_i ),   Δq_i = goal_joint(i) - q_now(i)
   ```

   系数 1.875 = 15/8 是 `kdl_quintic.hpp` 文件头给出的"两端 v=a=0 时峰值速度"
   `|v|max = 1.875·Δq/T` 的反解。**这是本任务唯一不用拍脑袋的默认值**。
   若 `joint_limits.check_velocity()` 为假（调用者没设速度上限），则拒绝自动定时并返回
   `"未给 duration 且未设置关节速度上限，无法自动定时"`。

**注意**：`buildQuinticTrajectory` 内部已经做过一次 `validateLimits()`，所以
`success == true` 就意味着这条轨迹在给定约束下可行——control 侧不要重复做同一件事，
只需把 message 透传。

---

## 5. 任务2：笛卡尔空间运动

`CartesianSpaceTask::solve(ctx, req, q_now, qdot_now)`：

| 步骤 | 动作 | 调用的底层接口 |
|---|---|---|
| 1 | 起点位姿：`req.has_start_pose` 用 `req.start_pose`；否则 `forwardKinematics(chain, q_now, T_now)` | `kdl_kinematics::forwardKinematics` |
| 2 | 组装路点序列：`cartesian_waypoints` 非空用它，否则单段 `{T_now, goal_pose}`；并用 `durations` / `duration` / 自动估算得到各段时长 | 同 §4（笛卡尔版解析峰值） |
| 3 | 构笛卡尔轨迹 | `kdl_interpolation::buildCartesianTrajectory(waypoints, durations, req.behavior, ctx.cartesian_limits)` |
| 4 | 按 `sample_dt` 采样出参考位姿序列与时刻 | `CartesianTrajectory::sample(t, CartesianState &)` |
| 5 | 逐点 IK（热启动：初值 = 上一点解） | `kdl_kinematics::solveIkLma(chain, q_init, state.pose, 1e-5, 500)` |
| 6 | 解连续性检查：`max_i |q_k - q_{k-1}| > req.joint_jump_threshold` → 失败 | — |
| 7 | 用采样点**重建关节轨迹**：路点 `q_k`、段时长 `sample_dt` | `kdl_interpolation::buildQuinticTrajectory` |
| 8 | 精度自检：重建轨迹在 `t_k` 采样 → FK → 与笛卡尔参考比较 | `QuinticTrajectory::sample` + `kdl_kinematics::forwardKinematics` |
| 9 | 自检超差 → `sample_dt` 减半重算一次；仍超差 → 失败并建议"增加中间路点" | — |
| 10 | 填 `ControlResult`（轨迹、`position_error`、`orientation_error`） | — |

**时间参数自动估算（第 2 步）**：与 §4 同源，用 `kdl_cartesian.hpp` 文件头给出的解析峰值：

```
位置：T >= 1.875 * |Δp| / v_max       （kStop 模式的单段解析峰值）
姿态：T >= 1.875 * theta   / ω_max     （slerp 段的解析峰值，theta 为等效转角）
取两者较大者。
```

**"不一定沿直线"怎么落地**：单段的语义固定是"位置走直线 + 姿态绕固定轴转最短弧"。
要弧线、绕行或经过中间构型，就给 `cartesian_waypoints` 补中间路点；`behavior` 选
`kStop`（每个路点精确停住，位置与姿态行为一致，默认）或 `kPassThrough`（位置路过内部路点
不减速，但**姿态在两种模式下都会停住**——这是 slerp 段"绕固定轴转"带来的硬约束，
详见 `kdl_cartesian.hpp` 文件头第五节）。

**为什么不用"雅可比速度映射"直接拼关节轨迹**（必须写进实现注释，否则后来者一定会问）：

* `QuinticTrajectory` 只能由 `buildQuinticTrajectory` 产生（构造函数私有、入参是路点 + 时长），
  要把任意 `(q, q̇, q̈)` 塞进去就必须修改 `Interpolation` 模块 —— 违反 §1 的边界 1；
* 采样足够密时，"采样 + IK + 关节侧重建"的位置偏差是 `O(sample_dt⁴)`（五次插值），
  而且第 8 步把这个偏差**量化出来了**，比"用雅可比算出来的 q̇ 更准"这种没有依据的说法更可靠。

**代价要说清楚**：重建后关节轨迹的**速度**是由五次插值重新解出来的，与"把笛卡尔速度经
`cartesianToJointVel` 映射到关节侧"得到的值有 `O(sample_dt²)` 差异。需要更贴近时减小
`sample_dt`（默认 0.01 s，与 `ros2_control` 的 100 Hz 控制周期对齐）。

---

## 6. 动力学（Dynamics）在哪里被调用

`plan` 要求"调用 Dynamics 中的底层算法"。落点选在**路由的公共收尾**，理由是两条任务
产出的都是关节轨迹，前馈计算与任务无关：

1. 采样：按 `sample_dt` 在 `joint_trajectory` 上取 `(t, q, q̇, q̈)`（`QuinticTrajectory::sample`）；
2. 逐点：`kdl_dynamics::inverseDynamics(chain, q, qdot, qddot)` → `τ`；（可选的外部力版本
   `inverseDynamicsWithTipWrenchBase` 作为后续扩展点，用来算"末端负重时各关节需要多大扭矩"）
3. 填 `torque_feedforward` / `torque_times`；
4. `req.check_torque_limit` 为真时，逐点比较 `|τ_i| > max_torque_i`：超限 → 结果标为失败并
   指出"第 k 个采样点（t=…s）第 i 个关节需要 … N·m，上限 … N·m"；只开前馈不检查则仅记录。

顺带的**健全性自检**（一次就够，放 `loadContextFromUrdf` 之后）：取当前静置位形调用
`inverseDynamics(chain, q, 0, 0)`，其 τ 应与 `kdl_dynamics::gravityTorque()` 一致且非零；
若全为 0 说明链上的段没有惯量参数（URDF 缺 `<inertial>`），此时前馈没有意义，直接报错提示。

---

## 7. 构建与安装（根 `CMakeLists.txt` 追加）

```cmake
# ---------------------------------------------------------------------------
# 任务层：src/control —— 调用 Tools/Kinematics/Interpolation/Dynamics 完成任务解算
# ---------------------------------------------------------------------------
add_library(kdl_control_lib
  src/control/src/router/kdl_task_base.cpp
  src/control/src/router/kdl_task_router.cpp
  src/control/src/router/kdl_control_print.cpp
  src/control/src/tasks/kdl_joint_space_task.cpp
  src/control/src/tasks/kdl_cartesian_space_task.cpp
)

# 只需要本模块的 include 根：四个依赖模块的 include 目录会随
# target_link_libraries(PUBLIC ...) 传递过来（它们都是 BUILD_INTERFACE 导出）。
target_include_directories(kdl_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/control/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

# 依赖方向：control 依赖全部四个模块；反向不允许（底层模块不认识 control）。
target_link_libraries(kdl_control_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
  kdl_tools_lib
  kdl_kinematics_lib
  kdl_interpolation_lib
  kdl_dynamics_lib
)
```

同时需要修改的既有位置：

* `install(TARGETS ...)` 列表里加 `kdl_control_lib`；
* 头文件安装：`install(DIRECTORY src/control/include/ DESTINATION include/kdl_tools)`
  （**保留 `router/`、`tasks/` 子目录**，否则消费者路径与源码树不一致）；
* `ament_export_libraries(...)` 加 `kdl_control_lib`；
* 示例：

  ```cmake
  add_executable(control_demo example/control_demo.cpp)
  target_compile_definitions(control_demo PRIVATE
    KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model")
  target_link_libraries(control_demo PRIVATE kdl_control_lib)
  ```

  并把它加进 `install(TARGETS ... RUNTIME DESTINATION lib/${PROJECT_NAME})`。

**不要**让 `kdl_control_lib` 依赖任何 ROS 消息类型（`trajectory_msgs` 等）：控制库保持
"只依赖 KDL + rclcpp 日志"，与 `ros2_control` 的桥接放在示例或 `src/ros2_control` 层。

---

## 8. 示例与验证（`example/control_demo.cpp`）

一个可执行文件串起全部验收点，输出既打印也导出 CSV（沿用 `quintic_demo` / `cartesian_demo`
的做法）：

1. `TaskRouter::loadContextFromUrdf(KDL_TOOLS_MODEL_DIR "/robotic_arm.urdf", ...)`，打印上下文
   （关节名、限位、可用约束项）；
2. **分派健壮性**：`type = kUnknown` → 打印"未实现"message；`q_now` 长度故意给错 → 打印尺寸
   错误的 message；两个用例都必须返回失败而不是崩溃；
3. **任务1**：从零位到 `[0.5, -0.4, 0.6, 0.3, 0.2, 0.5]`。打印 `TrajectoryResult`、
   `printKnotTable`；并打印"自动估算的 T"与 `1.875·|Δq_i|/v_max_i` 的逐关节对照；
4. **任务2**：两个算例 —— ①单段 位姿目标；②三路点（中间点偏离直线，用来演示"不一定走直线"），
   `kStop` 与 `kPassThrough` 各跑一次。每例都打印 `position_error` / `orientation_error`；
5. **动力学前馈**：`compute_torque_feedforward = true` 跑一次任务1，打印 τ 的峰值，并与
   `kdl_dynamics::gravityTorque()` 在起点处的值对照（静置时应量与重力项同量级）；
6. 导出 CSV：`control_joint_trajectory.csv`（t, q1..q6, q̇, q̈）、
   `control_cartesian_check.csv`（t, 参考位姿, FK 位姿, 残差），供后续画图脚本使用。

---

## 9. 分阶段实施与验收标准

每个阶段结束时**必须能 `colcon build --paths .` 通过**，并且阶段内的验收可复现。

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| **P0** | 目录骨架 + 空实现 + CMake target + `control_demo` 能跑起来 | 构建通过；`ros2 run kdl_tools control_demo` 正常退出 |
| **P1** | `TaskBase` / `TaskRouter` / `RobotContext` / `loadContextFromUrdf` / print | 示例第 1、2 项通过：上下文装配成功后关节名与限位打印正确；未实现任务与尺寸错误都有中文 message |
| **P2** | 任务1（关节空间） | 示例第 3 项通过：轨迹有效、`knotVelocities()` 两端为 0、自动定时与 `1.875Δq/v_max` 对照一致；把速度上限调小 10 倍 → 返回"某关节超速"而不是静默成功 |
| **P3** | 任务2（笛卡尔空间） | 示例第 4 项通过：单段与三路点各跑通，`position_error < 1e-3 m`、`orientation_error < 8.7e-3 rad`；目标给出工作空间外 → 报"第 k 个采样点 IK 未收敛" |
| **P4** | 动力学前馈 + 力矩上限检查 | 示例第 5 项通过；把 `max_torque` 人为调小 → 返回超限并指明确切关节与峰值 |
| **P5（可选）** | 与 `src/ros2_control` 桥接：把 `ControlResult` 采样成控制点（position/velocity/effort）发布给力矩控制器 | 在 MuJoCo 里 `error_code=0`（终态残差 < 1e-4 rad）。**注意：P5 已从「JTC 位置轨迹」改为「力矩控制」**，见 `src/ros2_control/PLAN_TORQUE.md` 与 `力矩控制问题报告.md`；旧的 `control_bridge_node` + `arm_controller` 已退休 |

---

## 10. 风险、取舍与扩展点

**风险与对策**

| 风险 | 对策 |
|---|---|
| IK 解跳变（肘部翻转、绕奇异） | 热启动 + `joint_jump_threshold` 拦截；报错时给出发生跳变的时刻与关节 |
| 采样式重建的精度不足 | 自检残差量化 + 自动将 `sample_dt` 减半重算一次（最多一次，避免无界循环）；总采样点数设上限保护（建议 ≤ 5000） |
| 自动定时公式的适用范围 | 解析峰值只对"单段、两端停住"严格成立；`kPassThrough` 下改用采样估峰值，并在注释与打印里注明这是近似 |
| 力矩前馈依赖惯量参数 | `loadContextFromUrdf` 后做一次静置 τ 自检（§6），惯量为 0 时明确报错 |
| 多段关节空间任务 | 本期只支持单段（`durations` 取首元素）；扩展方式是允许 `req.goal_joint` 改为"路点数组"——接口上预留，不改签名 |

**后续任务扩展点**（保持"加任务不改路由"）：

1. 新增 `tasks/kdl_*.hpp/.cpp`，实现 `TaskBase`；
2. 在 `TaskType` 里加枚举值；
3. 在 `TaskRouter` 构造函数里 `registerTask(...)`（或由调用者 `registerTask` 注入）。

例如后续计划：圆弧/样条笛卡尔任务、`minSingularValue` 奇异预警（`kdl_kinematics::minSingularValue`）、
末端外力下的力矩评估（`inverseDynamicsWithTipWrenchBase`）、以及用
`kdl_fdynamics::forwardDynamics` + RK4 做"轨迹跟随性"离线仿真校验。

---

## 附录 A：本计划用到的底层接口索引（实现时直接查）

**Tools（`kdl_tools.hpp`）**

```cpp
bool buildChainFromUrdfFile(const std::string & urdf_file, KDL::Chain & chain,
                            const std::string & base_link = kDefaultBaseLink,
                            const std::string & tip_link  = kDefaultTipLink);
bool loadUrdfModel(const std::string & urdf_file, urdf::Model & model);
constexpr const char * kDefaultBaseLink = "base_link";
constexpr const char * kDefaultTipLink  = "link6";
```

**Kinematics**

```cpp
bool forwardKinematics(const KDL::Chain &, const KDL::JntArray & q, KDL::Frame & frame);
bool forwardKinematicsAllSegments(const KDL::Chain &, const KDL::JntArray & q,
                                  std::vector<KDL::Frame> & frames);
bool computeJacobian(const KDL::Chain &, const KDL::JntArray & q, KDL::Jacobian & jacobian);
double minSingularValue(const KDL::Chain &, const KDL::JntArray & q);
IkResult solveIkLma(const KDL::Chain &, const KDL::JntArray & q_init, const KDL::Frame & target,
                    double eps = 1e-5, unsigned int max_iter = 500);
IkResult solveIkNrJl(const KDL::Chain &, const KDL::JntArray & q_init, const KDL::Frame & target,
                     const KDL::JntArray & q_min, const KDL::JntArray & q_max,
                     unsigned int max_iter = 10000, double eps = 1e-6);
// IkResult { KDL::JntArray q; int error_code; std::string message; bool success() const; }
VelocityResult     cartesianToJointVel(const KDL::Chain &, const KDL::JntArray & q, const KDL::Twist &);
VelocityResult     cartesianToJointVelDamped(const KDL::Chain &, const KDL::JntArray & q, const KDL::Twist &);
AccelerationResult cartesianToJointAcc(const KDL::Chain &, const KDL::JntArray & q,
                                       const KDL::JntArray & qdot, const KDL::Twist &);
bool jointToCartesianVel(const KDL::Chain &, const KDL::JntArray & q,
                         const KDL::JntArray & qdot, KDL::Twist & twist);
```

**Interpolation**

```cpp
TrajectoryResult buildQuinticTrajectory(const std::vector<KDL::JntArray> & waypoints,
                                        const std::vector<double> & durations,
                                        const JointLimits & limits = JointLimits());
CartesianResult  buildCartesianTrajectory(const std::vector<KDL::Frame> & waypoints,
                                          const std::vector<double> & durations,
                                          WaypointBehavior behavior = WaypointBehavior::kStop,
                                          const CartesianLimits & limits = CartesianLimits());

class QuinticTrajectory {
  bool valid() const; unsigned int joints() const; unsigned int segmentCount() const;
  double duration() const; double segmentDuration(unsigned int) const;
  const std::vector<KDL::JntArray> & waypoints() const;
  const std::vector<KDL::JntArray> & knotVelocities() const;
  const std::vector<KDL::JntArray> & knotAccelerations() const;
  const std::vector<double> & knotTimes() const;
  const QuinticCoefficients & segmentCoefficients(unsigned int seg, unsigned int joint) const;
  bool sample(double t, KDL::JntArray & q, KDL::JntArray & qdot, KDL::JntArray & qddot) const;
  bool sampleDerivatives(double t, KDL::JntArray & jerk, KDL::JntArray & snap) const;
  bool validateLimits(const JointLimits & limits, std::string & message) const;
};

class CartesianTrajectory {
  bool valid() const; unsigned int segmentCount() const; double duration() const;
  double segmentDuration(unsigned int) const; WaypointBehavior waypointBehavior() const;
  const std::vector<KDL::Frame> & waypoints() const; const std::vector<double> & knotTimes() const;
  const std::array<QuinticCoefficients, 3> & positionCoefficients(unsigned int seg) const;
  const SlerpSegment & orientationSegment(unsigned int seg) const;
  bool sample(double t, CartesianState & state) const;
  bool validateLimits(const CartesianLimits & limits, std::string & message) const;
};
// CartesianState { KDL::Frame pose; KDL::Vector linear_velocity, angular_velocity,
//                  linear_acceleration, angular_acceleration; }
// JointLimits     { max_velocity, max_acceleration, max_jerk, check_*() }
// CartesianLimits { max_linear_velocity, max_linear_acceleration,
//                   max_angular_velocity, max_angular_acceleration, check_*() }
// 峰值系数（用在自动定时）: 位置/关节 |v|max = 15Δ/(8T) = 1.875Δ/T
//                           姿态      |ω|max = 15θ/(8T) = 1.875θ/T
void quinticSmoothStep(double tau, double & s, double & ds, double & dds);
```

**Dynamics**

```cpp
IdResult inverseDynamics(const KDL::Chain &, const KDL::JntArray & q, const KDL::JntArray & qdot,
                         const KDL::JntArray & qddot,
                         const KDL::Vector & gravity = defaultGravity());
IdResult inverseDynamicsWithTipWrenchBase(const KDL::Chain &, const KDL::JntArray & q,
                                          const KDL::JntArray & qdot, const KDL::JntArray & qddot,
                                          const KDL::Wrench & tip_wrench_base,
                                          const KDL::Vector & gravity = defaultGravity());
// IdResult { KDL::JntArray torque; int error_code; std::string message; bool success() const; }
MassResult     jointSpaceInertia(const KDL::Chain &, const KDL::JntArray & q,
                                 const KDL::Vector & gravity = defaultGravity());
CoriolisResult coriolisTorque(const KDL::Chain &, const KDL::JntArray & q,
                              const KDL::JntArray & qdot, const KDL::Vector & gravity = defaultGravity());
GravityResult  gravityTorque(const KDL::Chain &, const KDL::JntArray & q,
                             const KDL::Vector & gravity = defaultGravity());
FdResult       forwardDynamics(const KDL::Chain &, const KDL::JntArray & q,
                               const KDL::JntArray & qdot, const KDL::JntArray & torque,
                               const KDL::Vector & gravity = defaultGravity());
```

---

## 附录 B：代码风格约定（与现有模块保持一致）

* 注释讲"为什么"，不讲"是什么"；单位写进注释（rad / rad·s⁻¹ / m / N·m）。
* 所有对外函数都有 `@brief/@param/@return` 等 doxyen 风格标签，失败路径的语义要写清楚。
* 结果结构体的字段带默认值，`success`/`message` 与底层模块同构。
* 不在教学库里写"防御性死循环/自动重试 N 次"之类的隐式魔法；失败就如实返回，
  由调用者决定下一步（与 `buildQuinticTrajectory` 的"不 clamp、不自动拉长时间"同一立场）。
* 一个 `.cpp` 只放本文件声明的东西；跨文件共享的辅助函数放匿名 namespace 或
  `include/router/`、`include/tasks/` 下的细节头文件。

---

## 附录 C：实现记录（与 §1–§10 计划的偏差、实测数据与踩到的坑）

本节是"实现后补记"，本文件仍然是接口的唯一出处：**先看这里，再改代码**。

### C.1 相对计划新增/调整的接口

| 项目 | 变化 | 原因 |
|---|---|---|
| `TaskRequest::rebuild_dt`（新增，默认 0.1 s） | 任务2 重建关节轨迹的路点间距 | 见 C.3：用 `sample_dt`（10 ms）当重建路点会让 jerk 虚高 1000 倍 |
| `ControlResult::min_singular_value`（新增） | 任务2 路径上雅可比最小奇异值的最小值 | 用来量化"离奇异面多远"，也是解算拒绝的依据 |
| `ControlResult::joint_trajectory` 语义微调 | 因**力矩超限**或**FK 自检超差**失败时仍保留轨迹 | 这两类失败下轨迹是完整的，丢掉只会让排查更难 |
| `JointSpaceTask::estimateDuration` / `CartesianSpaceTask::estimateDuration` | 提为 public static | 示例要能"把自动定时的来路并排打出来"给读者核对 |
| `TaskRouter::checkDynamicsAvailable` | 提为 public static | 让调用方能在跑之前先确认链上确实有惯量参数 |
| 关节空间任务的 `durations` | 长度必须为 1（收到 >1 直接报错） | 目前只支持单段，静默只用第一段会掩盖调用错误 |

### C.2 实测：P1–P4 验收结果（示例输出摘要）

* **P1 上下文装配**：`arm_hw`（6 关节 / 6 段）装配完整；行程 `[-3.14,3.14] … [-0.4,0.4]`、
  力矩上限 `[50,50,50,100,50,50]` 与 URDF 一致；动力学自检通过。
* **P1 健壮性**：未实现任务 → "任务类型未实现…"；关节角长度少 1 → "长度 5 与链的关节数 6 不匹配"；
  目标越界 → "第 3 个关节的目标 3.8 rad 超出行程 [-2.8, 2.8] rad"。三个都返回失败而非崩溃。
* **P2 任务1**：自动定时逐关节对照 `1.875·|Δq_i|/v_max_i` = `[0.9375, 0.375, …]`，取最大 0.9375 s
  （×1.02 余量 = 0.95625 s）；轨迹两端速度/加速度精确为 0；
  负向用例（固定 0.05 s）→ "第 0 段、关节 0 的速度峰值 18.75 rad/s 超过上限 1.0 rad/s"。
* **P3 任务2**（起点取非奇异构型 `[0,-0.6,0.8,0,0.2,0]`）：
  * 单段（自动定时，位置走直线）：成功，**位置残差 4.8e-6 m、姿态残差 1.04e-3 rad**，路径最小奇异值 0.123；
  * 三路点 kStop：成功，残差 6.0e-6 m / 1.00e-3 rad；
  * 三路点 kPassThrough：成功，残差 3.2e-6 m / 1.04e-3 rad，且参考路径**相对起止直线偏离 0.055 m**
    （"不沿直线运动"确实发生了）；
  * 负向用例（起点取零位＝直臂奇异位形）→ 立即判定"位形接近奇异：最小奇异值 0.00108 < 0.01"。
* **P4 动力学**：任务1 的力矩前馈峰值 `[2.55, 14.58, 7.42, 0.69, 0.054, 1.8e-4] N·m`；
  起点静止时的重力项 `[0, 12.06, -6.24, 0.56, -0.004, 0]` 与前馈首点同量级（自洽）；
  负向用例（上限压到 1 N·m）→ "t = 0 s，第 2 个关节需要 12.0628 N·m，上限 1 N·m"。

### C.3 踩到的两个坑（都已写进代码注释，别再踩）

**坑一：拿 `sample_dt` 直接当重建路点，jerk 会虚高上千倍。**
`buildQuinticTrajectory` 是解 C⁴ 连续条件得到路点速度/加速度的，而这些条件依赖路点位置的
高阶差分（jerk ~ 1/T³、snap ~ 1/T⁴）。用"已知平滑轨迹 + 高斯噪声"实测（真值 jerk 1.33 rad/s³）：

| 路点间距 | 噪声 1e-6 | 噪声 1e-5 | 噪声 1e-4 |
|---|---|---|---|
| 10 ms | 76 | **1064** | 4526 |
| 25 ms | 4.7 | 59 | 640 |
| 50 ms | 1.7 | 7.0 | 66 |
| 100 ms | 1.4 | **2.2** | 10 |
| 200 ms | 1.34（≈真值） | 1.4 | 2.1 |

逆解的关节角精度在 1e-5 rad 量级，所以 10 ms 的路点间距会把噪声放大成"必然超限"的假 jerk。
结论：**采样（`sample_dt`）与重建（`rebuild_dt`）必须分成两个网格**，默认 10 ms / 100 ms。

**坑二：路径经过奇异位形时，逐点逆解本身就不连续。**
`kdl_ik` 的 LMA 只保证**任务空间**误差（且姿态权重只有 0.01），在近奇异位形下关节解可以沿
零空间方向大幅摆动而不改变末端位姿：实测 10 ms 内关节跳 0.017 rad，而末端只动了 9e-5 m。
这种跳变经过任何插值都会被放大成大速度/大加速度，最后报出来的是"轨迹不可行"这种看不出真因的错。
所以本实现加了两个东西：①**近奇异判据**（`minSingularValue < 0.01` 直接拒绝并说明原因）；
②示例刻意不用零位当起点，并把"零位起步"做成负向用例（4c）。

标定数据：零位（直臂）= 0.0011，弯曲构型 = 0.159，目标构型 = 0.123。

**附带一个小坑**：自动定时用的是解析峰值，而插值模块的速度/加速度校验是密集采样取峰值，
恰好卡在上限上会报"峰值 0.500000 超过上限 0.500000"。所以 `estimateDuration` 末尾乘了
1.02 的安全余量（见两个任务里的 `kDurationSafetyFactor`）。
