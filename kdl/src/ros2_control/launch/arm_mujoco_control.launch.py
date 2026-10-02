"""启动 MuJoCo 仿真 + ros2_control 的机械臂控制链路。

链路顺序（也是本文件里节点的排列顺序）：
  1. robot_state_publisher  —— 把 robot_description(xacro 展开的 URDF) 发布到
                               /robot_description 话题，并按 /joint_states 广播 TF
  2. ros2_control_node      —— mujoco_ros2_control 自带的 controller_manager，
                               加载 MujocoSystemInterface 插件（"硬件"就是 MuJoCo），
                               按 update_rate=500 跑 read→update→write
  3. spawner                —— 加载并激活 joint_state_broadcaster、kdl_effort_controller
  4. rviz2（可选）          —— 看 TF / 机器人模型

力矩控制版：MJCF 用 scene_torque.xml（<motor> 执行器），关节命令接口是 effort。
参考轨迹由 kdl_control_node 发到 /control_reference（见 arm_mujoco_control_with_bridge.launch.py）。

用法：
  ros2 launch kdl_tools arm_mujoco_control.launch.py                 # 带 MuJoCo 窗口
  ros2 launch kdl_tools arm_mujoco_control.launch.py headless:=true  # 无界面
  ros2 launch kdl_tools arm_mujoco_control.launch.py rviz:=true      # 额外开 RViz

注意：所有节点都必须 use_sim_time=true。控制环靠 /clock（MuJoCo 每个物理步发布
一次）推进，这是"控制周期与仿真步长对齐"的关键，不要关掉。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("kdl_tools")

    # 在 OpaqueFunction 里一次性把所有替换解析成普通字符串，
    # 避免 launch 的 $(...) 与 xacro 的 $(...) / ${...} 互相干扰。
    resolve = context.perform_substitution

    xacro_exe = resolve(PathJoinSubstitution([FindExecutable(name="xacro")]))
    xacro_file = resolve(
        PathJoinSubstitution([pkg_share, "urdf", "robotic_arm_mujoco.urdf.xacro"])
    )
    controllers_file = resolve(
        PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    )

    # MJCF 入口：优先用 launch 参数，其次用包内安装好的 model/scene_torque.xml
    mujoco_model = resolve(LaunchConfiguration("mujoco_model")).strip()
    if not mujoco_model:
        mujoco_model = resolve(
            PathJoinSubstitution([pkg_share, "model", "scene_torque.xml"])
        )

    xacro_cmd = [
        xacro_exe,
        " ",
        xacro_file,
        " mujoco_model:=", mujoco_model,
        " headless:=", resolve(LaunchConfiguration("headless")),
        " sim_speed_factor:=", resolve(LaunchConfiguration("sim_speed_factor")),
    ]

    robot_description = {
        "robot_description": ParameterValue(
            Command(xacro_cmd), value_type=str
        )
    }

    nodes = []

    # --- 1. robot_state_publisher -------------------------------------------
    # 它同时把 URDF 以 transient_local QoS 发到 /robot_description，
    # ros2_control_node 就是从那个话题取 robot_description 的。
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # --- 2. 控制节点（必须用 mujoco_ros2_control 包里那个改造版） -----------
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        )
    )

    # --- 3. 控制器加载/激活 --------------------------------------------------
    # 顺序有讲究：先广播器，控制器才有状态可读。
    # --controller-manager-timeout 放大到 60 s：首次加载 MJCF + STL 网格较慢，
    # 而且 CM 还要等仿真时钟跑起来（wait_until_started）才会响应。
    for controller in ["joint_state_broadcaster", "kdl_effort_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--param-file",
                    controllers_file,
                    "--controller-manager-timeout",
                    "60",
                ],
                output="both",
            )
        )

    # --- 4. RViz（可选；机器人模型要能显示，URDF 里的网格路径必须能解析） ----
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="both",
            arguments=[
                "-d",
                resolve(PathJoinSubstitution([pkg_share, "rviz", "arm_mujoco.rviz"])),
            ],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
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
            OpaqueFunction(function=launch_setup),
        ]
    )
