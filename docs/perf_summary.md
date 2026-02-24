Natural Language
        ↓
LLM Parser (HTTP JSON)
        ↓
TaskOrchestrator (C++ rclcpp)
        ↓
RobotAdapter (抽象接口)
        ↓
TurtleBot3Adapter
        ↓
Nav2 / Camera / Gazebo  

自动跑：

BestEffort 30s

Reliable 30s

结果：

best_effort avg: 14.68 ms
reliable avg: 14.52 ms

分析结论：

在单机仿真环境下，两种 QoS 延迟差异不明显。
但在网络环境下 Reliable 会因重传增加延迟。