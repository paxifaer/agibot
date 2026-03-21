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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# add_executable(task_orchestrator
#   src/task_orchestrator.cpp
# )
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

# target_link_libraries(
#   task_orchestrator
#   nlohmann_json::nlohmann_json
#   CURL::libcurl
# )

add_executable(image_latency_monitor
  src/image_latency_monitor.cpp
)

find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(bt_main src/bt_main.cpp src/bt_nodes/navigate_node.cpp src/bt_nodes/observe_node.cpp)
# target_link_libraries(bt_main ${OpenCV_LIBRARIES})

add_library(adapter
  src/tiago_adapter.cpp
  src/turtlebot3_adapter.cpp
)
ament_target_dependencies(bt_main behaviortree_cpp_v3 rclcpp nav2_msgs rclcpp_action sensor_msgs cv_bridge)

ament_target_dependencies(
  adapter
  rclcpp
  rclcpp_action
  nav2_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(bt_main
  ${OpenCV_LIBRARIES}
  adapter
)


ament_target_dependencies(image_latency_monitor
  rclcpp
  sensor_msgs
)

# ament_target_dependencies(task_orchestrator
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   nav2_msgs
#   sensor_msgs
#   cv_bridge
# )
# ament_target_dependencies(bt_main
#   rclcpp
#   behaviortree_cpp_v3
# )

install(TARGETS
  bt_main
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  image_latency_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
