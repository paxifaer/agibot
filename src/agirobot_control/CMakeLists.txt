cmake_minimum_required(VERSION 3.8)
project(agirobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# include
include_directories(include)

# 添加可执行文件
add_executable(task_orchestrator src/task_orchestrator.cpp)
ament_target_dependencies(task_orchestrator
  rclcpp
  geometry_msgs
  nav2_msgs
)

# 安装可执行文件
install(TARGETS
  task_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
