# agibot
bot for agi
agirobot/
├── README.md
├── src/
│   ├── turtlebot3/             # 官方 / 自定义改动的 TurtleBot3 package
│   ├── turtlebot3_msgs/
│   ├── turtlebot3_simulations/ # Gazebo worlds / 仿真
│   ├── agirobot_control/       # 自己写的 LLM + ROS2 Action / Behavior Tree 逻辑
│   ├── agirobot_perception/    # 感知数据处理节点（LiDAR / 相机 / IMU）
│   └── agirobot_demo/          # 最终演示 Launch / Behavior Tree / Example Tasks
├── launch/                     # 所有 launch 文件
├── maps/                       # 测试地图
├── worlds/                     # Gazebo 自定义世界
└── scripts/                    # 辅助脚本（比如生成任务、测试脚本）

建图：
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true 
然后用 键盘或导航 让机器人绕场景走一圈。
ros2 run turtlebot3_teleop teleop_keyboard(使用键盘建图)

保存自己地图：ros2 run nav2_map_server map_saver_cli \
  -f my_map  

运行使用自己的world进行仿真：
  ros2 launch agirobot_control agibot_sim.launch.py world:=/home/parsifal/study/agibot/worlds/turtlebot3_world.world use_sim_time:=true

  ：质量尽量小，惯量尽量小但非零，保证仿真稳定。