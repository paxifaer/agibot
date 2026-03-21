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

拉取gazebo+navigation2：
  
  ros2 launch turtlebot3_gazebo  turtlebot3_world.launch.py
  ros2 launch turtlebot3_navigation2  navigation2.launch.py use_sim_time:=true 
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  log_level:=debug
  

运行使用自己的world进行仿真：
  ros2 launch agirobot_control agibot_sim.launch.py world:=/home/parsifal/study/agibot/worlds/turtlebot3_world.world use_sim_time:=true

  ：质量尽量小，惯量尽量小但非零，保证仿真稳定。
                        
turtlebot3 modelel:
  ros2 run agirobot_control bt_main /home/parsaifal/work/agibot/behavior_tree.xml --robot turtlebot3

tiago model:
     ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True                                                             
  
    ros2 run agirobot_control bt_main   /home/parsaifal/work/agibot/behavior_tree.xml  --robot tiago
```````````````````````Troubleshooting：TIAGo 仿真无法启动（Gazebo 崩溃）``````````````

问题现象：
运行 TIAGo 仿真时出现如下错误：
nouveau_pushbuf_data: Assertion `kref' failed.
gzserver exit code -6

或者：

* Gazebo 启动后直接崩溃
* 无法加载 world
* 仿真界面无法正常显示

问题原因：
该问题通常由显卡驱动引起，而不是 ROS2 或 TIAGo 配置问题。
使用的是 nouveau（开源 NVIDIA 驱动），在 Gazebo 渲染复杂模型（如 TIAGo）时容易崩溃。

解决方案（推荐）：
在启动前执行：

export LIBGL_ALWAYS_SOFTWARE=1

然后使用最小配置启动 TIAGo：

ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True arm_type:=no-arm end_effector:=no-end-effector ft_sensor:=no-ft-sensor moveit:=False tuck_arm:=False navigation:=True rviz:=False gzclient:=False

ros2 run topic_tools relay   /mobile_base_controller/odom /odom
说明：
该方法强制 Gazebo 使用 CPU 进行软件渲染，避免 GPU 驱动问题。

优点：

* 稳定，不会崩溃
* 不依赖显卡驱动
* 适用于服务器/无GPU环境

缺点：

* 渲染性能较低（但不影响功能验证）

可选方案：
如果需要 GPU 加速，可以安装 NVIDIA 官方驱动：

sudo ubuntu-drivers autoinstall
reboot

最终建议：
开发阶段统一使用：

export LIBGL_ALWAYS_SOFTWARE=1

可以保证 TIAGo 仿真稳定运行。

总结：
TIAGo 仿真在部分环境下会因显卡驱动问题导致 Gazebo 崩溃，可通过设置 LIBGL_ALWAYS_SOFTWARE=1 解决。
````````````````````````````````````````````````````````