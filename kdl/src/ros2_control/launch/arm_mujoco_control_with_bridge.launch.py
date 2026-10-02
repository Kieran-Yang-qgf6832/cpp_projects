"""在 arm_mujoco_control.launch.py 之上，再拉起 kdl_control_node（任务解算节点）。

链路（比原 launch 多出最后一环）：
  robot_state_publisher → ros2_control_node(MuJoCo 硬件)
                        → spawner(jsb + kdl_effort_controller)
                        → kdl_control_node（srv → 解算 → /control_reference）

为什么用"包含现有 launch + 追加一个节点"，而不是把节点塞进现有文件：
  现有 launch 是"仿真 + 硬件 + 控制器"的最小可复现链路。控制节点是可选的上层，
  单独一份文件既不碰那条链路的启动顺序，也省得把 headless/rviz 这些参数各抄一遍。

用法：
  ros2 launch kdl_tools arm_mujoco_control_with_bridge.launch.py headless:=true
  # 另开终端（launch 起完约 5~10 s）：
  ros2 service call /control_task kdl_tools/srv/ControlTask \
    "{task_type: 0, goal_joint: [0.5, -0.4, 0.6, 0.3, 0.2, 0.5], duration: 5.0}"

注意：控制节点会阻塞着等轨迹执行完（由控制器的 ~/status 回报），这是设计如此
（service 的语义就是"把这条轨迹执行完"）；它用多线程执行器，不影响 /clock 与控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kdl_tools"),
                    "launch",
                    "arm_mujoco_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mujoco_model": LaunchConfiguration("mujoco_model"),
            "headless": LaunchConfiguration("headless"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    # 控制节点：不需要等控制器起来再启动 —— 它是在收到 service 请求时才去查
    # /control_reference 与状态，早一点起反而能更早暴露 URDF / 关节映射的配置错误。
    control_node = Node(
        package="kdl_tools",
        executable="kdl_control_node",
        name="control_node",
        emulate_tty=True,
        output="both",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            # 这四个参数与 arm_mujoco_control.launch.py 同名同默认值，直接透传，
            # 这样 `headless:=true` 之类的用法在两个 launch 上完全一致。
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description="MJCF 入口文件；留空则用 share/kdl_tools/model/scene_torque.xml",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="true = 不开 MuJoCo 图形窗口",
            ),
            DeclareLaunchArgument(
                "sim_speed_factor",
                default_value="1.0",
                description="仿真时间/墙钟时间倍率，>0 时会锁住 MuJoCo 窗口的实时率",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="是否额外启动 RViz2",
            ),
            base_launch,
            control_node,
        ]
    )
