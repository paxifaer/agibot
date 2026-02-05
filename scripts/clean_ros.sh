#!/bin/bash

echo "==== Killing ROS2 / Gazebo / RViz ===="

# Kill ROS2 and launch
pkill -9 -f ros2
pkill -9 -f launch_ros
pkill -9 -f rviz

# Kill Gazebo
pkill -9 gzserver
pkill -9 gzclient

# Kill common python ROS nodes
pkill -9 -f robot_state_publisher
pkill -9 -f map_server
pkill -9 -f nav2
pkill -9 -f slam_toolbox

# Clear shared memory
echo "==== Clearing /dev/shm ===="
rm -rf /dev/shm/*

# Clear ROS2 cache
echo "==== Clearing ~/.ros ===="
rm -rf ~/.ros

# Clear RViz config
echo "==== Clearing ~/.rviz2 ===="
rm -rf ~/.rviz2

# Reset domain id
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

echo "==== ROS environment cleaned ===="
