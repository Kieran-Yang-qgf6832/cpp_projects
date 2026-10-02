cmake_minimum_required(VERSION 3.8)
project(kdl_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 教学包使用的都是 C++17（KDL 本身要求 >= 14）。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)        # urdf 包装层: urdf::Model / parseURDFFile
find_package(kdl_parser REQUIRED)        # kdl_parser: urdf -> KDL::Tree
find_package(orocos_kdl_vendor REQUIRED)  # orocos_kdl: KDL::Tree / KDL::Chain 本体

# ---- control 节点 / 力矩控制器插件用到的 ----
find_package(rosidl_default_generators REQUIRED)  # 本包自定义 srv/msg
find_package(geometry_msgs REQUIRED)      # ControlTask.srv 里的 geometry_msgs/Pose
find_package(std_msgs REQUIRED)           # ControlStatus.msg 里的 Header
find_package(ament_index_cpp REQUIRED)    # 按包名定位 share/kdl_tools/model/*.urdf
find_package(sensor_msgs REQUIRED)        # 订阅 /joint_states
find_package(trajectory_msgs REQUIRED)    # 参考轨迹 trajectory_msgs/JointTrajectory
find_package(controller_interface REQUIRED)  # 力矩控制器插件基类
find_package(pluginlib REQUIRED)             # 控制器插件导出
find_package(realtime_tools REQUIRED)        # RT 缓冲 / 实时状态发布
find_package(yaml-cpp REQUIRED)              # 解析 config/mujoco_pids.yaml

# ---------------------------------------------------------------------------
# 教学库：src/Tools 下的 URDF -> KDL 工具
# ---------------------------------------------------------------------------
add_library(kdl_tools_lib
  src/Tools/src/kdl_tools.cpp
  src/Tools/src/kdl_tools_print.cpp
)

# 本项目头文件规范：每个功能模块各自持有 include/ 与 src/，例如
#   src/Tools/include/kdl_tools.hpp   公共头（该模块的 include/ 即包含根）
#   src/Tools/src/kdl_tools.cpp       实现
# 因此构建期把 src/Tools/include 作为包含根，源码里直接 #include "kdl_tools.hpp"。
# 安装后统一落到 include/kdl_tools/，消费者路径保持不变。
# urdf 包只导出 urdf_INCLUDE_DIRS 变量，而且它是多元素列表，
# 不能包进 $<BUILD_INTERFACE:...>（分号会被当成参数分隔符），因此直接使用。
target_include_directories(kdl_tools_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Tools/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
  ${urdf_INCLUDE_DIRS}
)

target_link_libraries(kdl_tools_lib PUBLIC
  rclcpp::rclcpp
  kdl_parser::kdl_parser
  orocos-kdl
  # urdf 包只导出变量形式：需要显式链接 liburdf.so
  ${urdf_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 教学库：src/Kinematics 下的 FK / 雅可比 / IK
# ---------------------------------------------------------------------------
add_library(kdl_kinematics_lib
  src/Kinematics/src/kdl_fk.cpp
  src/Kinematics/src/kdl_jacobian.cpp
  src/Kinematics/src/kdl_ik.cpp
  src/Kinematics/src/kdl_velocity.cpp
  src/Kinematics/src/kdl_acceleration.cpp
  src/Kinematics/src/kdl_kinematics_print.cpp
)

target_include_directories(kdl_kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

# Kinematics 只依赖 KDL 本身，与 Tools / urdf 完全解耦：
# 它接受的是已经建好的 KDL::Chain，这正是"解耦"在链接层面的体现。
# Eigen 的包含路径由 orocos-kdl 的 INTERFACE 依赖(Eigen3::Eigen)带进来。
target_link_libraries(kdl_kinematics_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
)

# ---------------------------------------------------------------------------
# 教学库：src/Dynamics 下的逆动力学 / 正动力学 / M-C-G 分解
# ---------------------------------------------------------------------------
add_library(kdl_dynamics_lib
  src/Dynamics/src/kdl_idynamics.cpp
  src/Dynamics/src/kdl_dynparam.cpp
  src/Dynamics/src/kdl_fdynamics.cpp
  src/Dynamics/src/kdl_dynamics_print.cpp
)

# 除了本模块的 include/，还需要 Kinematics 的 include/：
# "基座坐标系下的末端外力"必须先做一次正运动学拿到末端姿态，
# 所以 Dynamics 在链接层面依赖 Kinematics（见 kdl_idynamics.cpp 的注释）。
target_include_directories(kdl_dynamics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Dynamics/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics/include>
)

target_link_libraries(kdl_dynamics_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
  kdl_kinematics_lib
)

# ---------------------------------------------------------------------------
# 教学库：src/Interpolation 下的插值
#   关节空间：五次多项式（kdl_quintic）
#   笛卡尔空间：位置五次多项式 + 姿态 slerp（kdl_cartesian）
# ---------------------------------------------------------------------------
add_library(kdl_interpolation_lib
  src/Interpolation/src/kdl_quintic.cpp
  src/Interpolation/src/kdl_interpolation_print.cpp
  src/Interpolation/src/kdl_cartesian.cpp
  src/Interpolation/src/kdl_cartesian_print.cpp
)

target_include_directories(kdl_interpolation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Interpolation/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

# 插值只吃"路点 + 段时长"，与 URDF、正逆运动学、动力学都无关：
# 关节空间给的是关节角数组，笛卡尔空间给的是 KDL::Frame 数组，两者都只依赖
# KDL 本身（kdl_cartesian 内部转调 kdl_quintic 的多段求解器，同属本模块，
# 不新增任何依赖）。
target_link_libraries(kdl_interpolation_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
)

# ---------------------------------------------------------------------------
# 任务层：src/control —— 调用上面四个模块完成"任务解算"
#   router/  任务基类 + 任务调度路由（按任务信号分派 + 统一的动力学收尾）
#   tasks/   具体任务：关节空间运动、笛卡尔空间运动
# 依赖方向是单向的：control → {Tools, Kinematics, Interpolation, Dynamics}，
# 底层模块完全不认识 control，所以这一层可以整体替换/删除而不影响教学库。
# ---------------------------------------------------------------------------
add_library(kdl_control_lib
  src/control/src/router/kdl_task_base.cpp
  src/control/src/router/kdl_task_router.cpp
  src/control/src/router/kdl_control_print.cpp
  src/control/src/tasks/kdl_joint_space_task.cpp
  src/control/src/tasks/kdl_cartesian_space_task.cpp
)

# 只需要本模块的 include 根：四个依赖模块的 include 目录会随 PUBLIC 链接传递
# 过来（它们都是 $<BUILD_INTERFACE:...> 导出），不必重复列。
target_include_directories(kdl_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/control/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

target_link_libraries(kdl_control_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
  kdl_tools_lib
  kdl_kinematics_lib
  kdl_interpolation_lib
  kdl_dynamics_lib
)

# ---------------------------------------------------------------------------
# 教学示例可执行文件
# ---------------------------------------------------------------------------
add_executable(build_from_urdf example/build_from_urdf.cpp)
# 把源码里的 model 目录路径编译进程序，示例即可在源码树下直接运行。
target_compile_definitions(build_from_urdf PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(build_from_urdf PRIVATE kdl_tools_lib)

# FK / 雅可比 / IK 示例：既用到 Tools（建链），也用到 Kinematics（求解）。
add_executable(fk_ik_demo example/fk_ik_demo.cpp)
target_compile_definitions(fk_ik_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(fk_ik_demo PRIVATE kdl_tools_lib kdl_kinematics_lib)

# 速度 / 加速度转换示例：演示 J·q̇、J̇·q̇ 与伪逆求逆。
add_executable(vel_acc_demo example/vel_acc_demo.cpp)
target_compile_definitions(vel_acc_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(vel_acc_demo PRIVATE kdl_tools_lib kdl_kinematics_lib)

# 动力学示例：逆解、正解、三明治公式拆分、末端外力与 RK4 仿真。
add_executable(dynamics_demo example/dynamics_demo.cpp)
target_compile_definitions(dynamics_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(dynamics_demo PRIVATE kdl_tools_lib kdl_dynamics_lib)

# 五次插值示例：单段系数、多段 C⁴ 拼接、约束校验与 CSV 导出。
# 刻意不定义 KDL_TOOLS_MODEL_DIR：插值只吃关节数组，不需要机器人模型。
add_executable(quintic_demo example/quintic_demo.cpp)
target_link_libraries(quintic_demo PRIVATE kdl_interpolation_lib)

# 笛卡尔插值示例：位置五次多项式 + 姿态 slerp、多段两种路点行为、限位校验与 CSV。
# 同样不定义 KDL_TOOLS_MODEL_DIR：笛卡尔插值只吃位姿数组，不需要机器人模型。
add_executable(cartesian_demo example/cartesian_demo.cpp)
target_link_libraries(cartesian_demo PRIVATE kdl_interpolation_lib)

# 任务层示例：URDF → RobotContext → 任务信号 → 路由解算 → 关节轨迹 + CSV。
# 它需要模型文件（起点位姿用 FK 求），所以同样定义 KDL_TOOLS_MODEL_DIR。
add_executable(control_demo example/control_demo.cpp)
target_compile_definitions(control_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(control_demo PRIVATE kdl_control_lib)

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS kdl_tools_lib kdl_kinematics_lib kdl_dynamics_lib kdl_interpolation_lib
        kdl_control_lib
        build_from_urdf fk_ik_demo vel_acc_demo dynamics_demo quintic_demo cartesian_demo
        control_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/Tools/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Kinematics/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Dynamics/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Interpolation/include/
  DESTINATION include/kdl_tools
)

# control 的 include/ 下有 router/ 与 tasks/ 两个子目录，安装时必须保持原样，
# 否则消费者写的 #include "router/kdl_task_router.hpp" 在安装后找不到。
install(DIRECTORY src/control/include/
  DESTINATION include/kdl_tools
)

# 把 URDF / STL 一起装到 share/kdl_tools/model，示例运行时按此路径查找。
install(DIRECTORY src/model/
  DESTINATION share/${PROJECT_NAME}/model
)

# ---------------------------------------------------------------------------
# ros2_control 层（MuJoCo 硬件接口 + 控制器）
#   urdf/   机器人描述 xacro：include 上面的 model，再挂 <ros2_control> 标签
#   config/ 控制器 / PID 参数
#   launch/ 启动文件（ros2launch 只在 share/<pkg>/launch/ 里找，路径不能改）
#   rviz/   可视化配置
# 这些都不参与编译，装到 share 下即可；xacro 里的 $(find kdl_tools) 依赖此处安装。
# 不装也没关系 —— 只要不跑 ros2 launch kdl_tools arm_mujoco_control.launch.py。
# ---------------------------------------------------------------------------
install(DIRECTORY src/ros2_control/urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/ros2_control/config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/ros2_control/rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/ros2_control/launch DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 通信接口：service（任务请求）+ msg（控制器状态回报）
#   src/communication/srv/ControlTask.srv          自定义 service
#   src/communication/msg/ControlStatus.msg        控制器 → 控制节点的状态回报
#
# 接口定义放在 src/communication/ 下（与 Tools、control 平级的"通信层"目录），
# 但它**属于 kdl_tools 这个包**：rosidl 生成的类型要被别的包消费时，才必须真的建
# 一个包（package.xml + CMakeLists）；这里只有本包自己用，所以随包生成即可。
# 路径是相对包根（CMAKE_CURRENT_SOURCE_DIR）的，不是 ${CMAKE_CURRENT_SOURCE_DIR}/../…
# —— 后者会指到工作区外面去。
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "src/communication/srv/ControlTask.srv"
  "src/communication/msg/ControlStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 取本包生成的 typesupport 目标：用官方推荐的 rosidl_get_typesupport_target，
# 不要写 target_link_libraries(... ${PROJECT_NAME}) —— 那个聚合目标在 Jazzy 上
# 不会把 typesupport 带进来，编译期就会报 "找不到 kdl_tools/srv/control_task.hpp"。
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# ---------------------------------------------------------------------------
# 力矩控制器插件（真正的"控制环内 bridge"）
#   src/ros2_control/include/kdl_effort_controller.hpp
#   src/ros2_control/src/kdl_effort_controller.cpp
# 由 controller_manager 以插件方式加载；在 500 Hz update() 内算 τ 并写 effort 接口。
# 依赖 kdl_tools_lib（建链 + URDF 限位）与 kdl_dynamics_lib（重力前馈）。
# ---------------------------------------------------------------------------
if(TARGET yaml-cpp::yaml-cpp)
  set(KDL_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(KDL_YAML_CPP_TARGET yaml-cpp)
endif()

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
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  trajectory_msgs::trajectory_msgs
  kdl_tools_lib
  kdl_dynamics_lib
  ${KDL_YAML_CPP_TARGET}
)
target_link_libraries(kdl_effort_controller "${cpp_typesupport_target}")

# 注册插件：安装后落在 share/kdl_tools/controller_plugins.xml，CM 靠它找到类名。
pluginlib_export_plugin_description_file(
  controller_interface src/ros2_control/controller_plugins.xml)

# ---------------------------------------------------------------------------
# 任务解算节点（control 层）
#   src/control/src/node/kdl_control_node.cpp
# srv → TaskRouter → 按控制周期离散（含逐点逆动力学前馈）→ /control_reference。
# ---------------------------------------------------------------------------
add_executable(kdl_control_node
  src/control/src/node/kdl_control_node.cpp
)
target_link_libraries(kdl_control_node
  kdl_control_lib                   # TaskRouter / TaskRequest / ControlResult
  kdl_dynamics_lib                  # inverseDynamics → τ_ff
  rclcpp::rclcpp                    # 节点、service、/joint_states、发布参考
  ament_index_cpp::ament_index_cpp  # 按包名定位 share/kdl_tools/model/robotic_arm.urdf
  sensor_msgs::sensor_msgs
  trajectory_msgs::trajectory_msgs
)
target_link_libraries(kdl_control_node "${cpp_typesupport_target}")

install(TARGETS kdl_effort_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS kdl_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 画图脚本：读 quintic_demo 导出的 CSV 画曲线
# ---------------------------------------------------------------------------
# 必须 RENAME 成不带扩展名的 plot_quintic：
# `ros2 run` 在 Linux 下要求 lib/<pkg>/ 里的文件名与命令名**完全一致**，
# 没有 .py 的回退（见 ros2run/api/__init__.py 的 get_executable_path）。
# install(PROGRAMS) 会顺手加上可执行权限，脚本里的 shebang 负责选解释器。
install(PROGRAMS src/Interpolation/scripts/plot_quintic.py
  RENAME plot_quintic
  DESTINATION lib/${PROJECT_NAME}
)

# 笛卡尔插值的画图脚本同理：读 cartesian_demo 导出的 CSV，出总览 + 分张 PNG。
install(PROGRAMS src/Interpolation/scripts/plot_cartesian.py
  RENAME plot_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include/kdl_tools)
ament_export_libraries(kdl_tools_lib kdl_kinematics_lib kdl_dynamics_lib kdl_interpolation_lib
                      kdl_control_lib)
ament_export_dependencies(rclcpp urdf kdl_parser orocos_kdl_vendor)

ament_package()
