#!/bin/bash

echo "Start performance test"

rm -f nav_latency.csv
rm -f observe_latency.csv

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &

sleep 10

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true &

sleep 10

ros2 run agirobot_control bt_main behavior_tree.xml

echo "Test finished"

python3 tools/analyze_latency.py