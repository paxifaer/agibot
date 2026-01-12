cmake_minimum_required(VERSION 3.8)
project(agirobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必须最先 find ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(task_orchestrator
  src/task_orchestrator.cpp
)
find_package(nlohmann_json REQUIRED)

target_link_libraries(
  task_orchestrator
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(task_orchestrator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  task_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
