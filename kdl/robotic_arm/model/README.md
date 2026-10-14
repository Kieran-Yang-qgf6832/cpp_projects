# robotic_arm 机械臂模型（命名与 src/model 完全一致）

本目录把 SolidWorks 导出的 robotic_arm 机械臂整理成与 `src/model/` 完全一致的文件布局与命名规则，
并补齐了同等的 MJCF。所有文件放在**同一个目录**，因此 MJCF 的 mesh 用相对路径
（`file="base_link.STL"`），与 `src/model/robotic_arm.xml` 的做法相同。

## 文件命名（与 `src/model/` 一模一样）

| 文件 | 说明 |
| --- | --- |
| `robotic_arm.urdf` | **纯模型**（SolidWorks 导出），运动学/惯量唯一真源 |
| `robotic_arm.xml` | 位置伺服版 MJCF（`<position>`，kp=100/800/500/50） |
| `robotic_arm_torque.xml` | 力矩版 MJCF（`<motor>`）—— 力矩控制走这一份 |
| `scene.xml` | 位置版场景入口，`include robotic_arm.xml` |
| `scene_torque.xml` | 力矩版场景入口，`include robotic_arm_torque.xml`，`timestep=0.002` |
| `robotic_arm.csv` | SolidWorks 导出原始参数表 |
| `*.STL` | 5 个连杆网格（与 `../meshes/` 内容一致） |

## 参数对齐（相对 `src/model/robotic_arm.urdf`）

- **关节限位 / effort / velocity**：逐条与参考模型一致

  | 关节 | lower | upper | effort | velocity |
  | --- | --- | --- | --- | --- |
  | joint1 | -1.5708 | 1.5708 | 50 | 10 |
  | joint2 | -3.1 | 3.1 | 50 | 10 |
  | joint3 | -3.14 | 3.14 | 50 | 10 |
  | joint4 | -1.5708 | 1.5708 | 50 | 10 |

- **mesh filename**：改为本包内的 `package://robotic_arm/model/<link>.STL`（原来指向
  `package://robotic_arm/meshes/`，与新的目录布局对应）。
- **根 link**：新增无惯量 `world` + 固定关节 `world_to_base`，与参考 URDF、
  以及 MJCF 的 `<worldbody><body name="base_link">` 结构对齐（消除 kdl_parser 的
  根 link 惯量警告）。`base_link` 自身的惯量未做任何改动。
- **质量 / 惯量 / 关节原点**：这些是 robotic_arm 自己的几何数据，**不**从参考模型复制，
  逐条来自 `robotic_arm.urdf`（并已确认各惯量张量物理有效）。

## MJCF 侧与参考模型逐项一致

- `compiler angle="radian" autolimits="true"`、`option integrator="implicitfast"`
- body 树 / 关节名 / 执行器名 / sensor 名与参考完全同构（4 关节、4 执行器、14 sensor）
- 关节阻尼分关节给值：joint1/2 = 0.2，joint3/4 = 0.02
- 各 joint 的 `range` = URDF 限位，`actuatorfrcrange="-50 50"` = URDF effort
- 位置版 kp = 100/800/500/50，`dampratio="1.0"`；力矩版 `gear="1"`、`ctrlrange="-50 50"`
- 自碰撞排除：位置版只排除相邻连杆；力矩版另外排除全部非相邻连杆（网格凸包近似会误判）
- `tool_site`（base_link）与 `ee_site`（link4, x=0.1165）与参考一致

## 校验方式

本目录的 MJCF 已用 MuJoCo 3.6.0 实际加载验证（`MjModel.from_xml_path` +
`mj_forward`/`mj_step`），两套 scene 均无警告加载，质量、限位、阻尼、执行器增益
与 URDF 逐项吻合。

## 关于旧文件

`../urdf/robotic_arm.urdf` 与 `../urdf/robotic_arm.csv` 是本次整理前的原始导出件，其中
`robotic_arm.urdf` 的关节限位/effort/velocity 均为 0、mesh 指向 `package://robotic_arm/meshes/`，
**已被本目录取代**。`launch/display.launch` 与 `launch/gazebo.launch` 已改为指向
`model/robotic_arm.urdf`；旧文件保留仅作对照，确认无误后可删除。
