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
