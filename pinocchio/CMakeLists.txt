cmake_minimum_required(VERSION 3.16)
project(pinocchio_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# 全局编译设置
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# 依赖
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------------------------
# Tool 模块库：URDF 解析 + 打印工具
# ---------------------------------------------------------------------------
add_library(pinocchio_tools_lib
  src/Tool/src/pinocchio_parser.cpp
  src/Tool/src/pinocchio_parser_print.cpp
)

target_include_directories(pinocchio_tools_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Tool/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# pinocchio::pinocchio 会传递 Eigen、coal、urdfdom、Boost 等头文件与链接需求。
target_link_libraries(pinocchio_tools_lib PUBLIC pinocchio::pinocchio)
target_compile_features(pinocchio_tools_lib PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# 示例
# ---------------------------------------------------------------------------
add_executable(parse_demo example/parse_demo.cpp)
target_link_libraries(parse_demo pinocchio_tools_lib ament_index_cpp::ament_index_cpp)

add_executable(fk_demo example/fk_demo.cpp)
target_link_libraries(fk_demo pinocchio_tools_lib ament_index_cpp::ament_index_cpp)

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS
    pinocchio_tools_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY src/Tool/include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# 示例可执行文件安装到 lib/${PROJECT_NAME}，从而可以通过 ros2 run 调用。
install(TARGETS
    parse_demo
    fk_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 机器人模型资源，例如 src/model/robotic_arm.urdf。
install(DIRECTORY src/model/
  DESTINATION share/${PROJECT_NAME}/model
)

# ---------------------------------------------------------------------------
# ament 导出
# ---------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)

ament_package()
