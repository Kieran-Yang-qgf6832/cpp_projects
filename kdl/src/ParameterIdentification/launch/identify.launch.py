"""启动 MuJoCo 仿真 + 参数辨识节点（kdl_identify_node）。

它复用 ros2_control 的仿真 launch（不动那份文件），再追加辨识节点：
  仿真 + 硬件 + joint_state_broadcaster + kdl_effort_controller
    → kdl_identify_node 自行优化激励轨迹、发 /control_reference、
      采 /mujoco_actuators_states、解参数、打印并写 CSV。

用法：
  ros2 launch kdl_tools identify.launch.py headless:=true
  ros2 launch kdl_tools identify.launch.py headless:=true auto_start:=false
  # 手动触发（auto_start:=false 时）：
  ros2 service call /kdl_identify/run std_srvs/srv/Trigger {}

注意：
  * kdl_effort_controller 的 feedforward 必须为 true（controllers.yaml 默认）。
    辨识节点会给参考点填"名义模型逆动力学"前馈，臂才会贴着参考动；
    这不影响辨识——辨识用的是 /mujoco_actuators_states 的**实测力矩**。
  * kdl_identify_node 与控制器都用 use_sim_time=true。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kdl_share = FindPackageShare("kdl_tools")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kdl_share, "launch", "arm_mujoco_control.launch.py"])
        ),
        launch_arguments={
            "mujoco_model": LaunchConfiguration("mujoco_model"),
            "headless": LaunchConfiguration("headless"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    identify_node = Node(
        package="kdl_tools",
        executable="kdl_identify_node",
        name="kdl_identify",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "auto_start": LaunchConfiguration("auto_start"),
                "lead_in_time": LaunchConfiguration("lead_in_time"),
                "warmup_periods": LaunchConfiguration("warmup_periods"),
                "measure_periods": LaunchConfiguration("measure_periods"),
                "period": LaunchConfiguration("period"),
                "harmonics": LaunchConfiguration("harmonics"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description="MJCF 入口；留空用 share/kdl_tools/model/scene_torque.xml",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", description="true = 不开 MuJoCo 窗口"
            ),
            DeclareLaunchArgument(
                "sim_speed_factor", default_value="1.0", description="仿真/墙钟时间倍率"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="是否额外启动 RViz2"
            ),
            DeclareLaunchArgument(
                "auto_start", default_value="true", description="仿真就绪后自动开始辨识"
            ),
            DeclareLaunchArgument(
                "lead_in_time", default_value="2.0", description="入场过渡时长 [s]"
            ),
            DeclareLaunchArgument(
                "warmup_periods", default_value="1.0", description="丢弃的前导周期数"
            ),
            DeclareLaunchArgument(
                "measure_periods", default_value="2.0", description="用于辨识的周期数"
            ),
            DeclareLaunchArgument(
                "period", default_value="4.0", description="激励轨迹周期 [s]"
            ),
            DeclareLaunchArgument(
                "harmonics", default_value="5", description="傅里叶谐波数"
            ),
            sim,
            identify_node,
        ]
    )
