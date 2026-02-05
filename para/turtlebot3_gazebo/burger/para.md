

### 1. DWB 控制器调试模式开启

位置：

```yaml
controller_server -> FollowPath
```

参数：

```yaml
debug_trajectory_details: True
```

说明：

* 默认一般为 False
* 当前开启用于调试轨迹评分
* 会增加日志输出

状态：保留

---

### 2. TurtleBot3 官方 Issue 补丁

位置：

```yaml
controller_server -> FollowPath
```

涉及参数：

```yaml
min_speed_xy
min_speed_theta
```

说明：

* 用于解决 TurtleBot3 低速抖动问题
* 来源于官方仓库 issue

状态：保留

---

### 3. AMCL 粒子数增强

位置：

```yaml
amcl
```

参数：

```yaml
min_particles: 500
max_particles: 2000
```

说明：

* 高于默认值
* 提高定位稳定性
* 增加 CPU 开销

状态：保留

---

### 4. Waypoint Follower 高频设置

位置：

```yaml
waypoint_follower
```

参数：

```yaml
loop_rate: 2000
```

说明：

* 默认一般 20~100
* 当前极高频率
* 用于提高路径点执行响应
* 可能增加 CPU 占用

状态：保留

---

### 5. Costmap Inflation 扩大

#### Local Costmap

```yaml
inflation_radius: 1.0
```

#### Global Costmap

```yaml
inflation_radius: 0.55
```

说明：

* 比默认更大
* 提高避障安全性
* 会减少可通行空间

状态：保留

---

### 6. 控制器频率限制

位置：

```yaml
controller_server
```

参数：

```yaml
controller_frequency: 10.0
```

说明：

* 相对较低频率
* 适合 Gazebo 仿真环境
* 提升稳定性

状态：保留

---

### 7. use_sim_time 全局开启

位置：多个节点

统一设置：

```yaml
use_sim_time: True
```

涉及节点：

* amcl
* controller_server
* planner_server
* bt_navigator
* costmap
* map_server
* robot_state_publisher

说明：

* 保证仿真时间同步
* 防止 TF / 控制异常

状态：必须保留

---

## 四、存在的潜在冲突点（已知风险）

### 1. 目标容差参数多处重复

涉及位置：

1. controller_server
2. general_goal_checker
3. FollowPath (DWB)

示例：

```yaml
controller_server:
  xy_goal_tolerance: 0.05

general_goal_checker:
  xy_goal_tolerance: 0.25

FollowPath:
  xy_goal_tolerance: 0.05
```

风险：

* 多套逻辑并存
* 可能导致到点后微抖
* 停止判定不一致

当前状态：暂不修改

---

### 2. 停止速度阈值偏大

位置：

```yaml
FollowPath
```

参数：

```yaml
trans_stopped_velocity: 0.25
```

说明：

* 判定“已停止”门槛偏高
* 可能导致低速抖动

当前状态：暂不修改

---

### 建议 1：统一目标精度

```yaml
xy_goal_tolerance: 0.05
yaw_goal_tolerance: 0.1
```

适用场景：

* 到点抖动
* 无法结束任务

---

### 建议 2：增加停止判定

```yaml
stopped_velocity_threshold: 0.02
trans_stopped_velocity: 0.02
```

适用场景：

* 微动不停
* 最后阶段振荡

---

### 建议 3：TF 延迟容忍

```yaml
transform_tolerance: 0.3
```

适用场景：

* TF 延迟
* queue full 报错



### 1. 配置管理建议

推荐目录结构：

```text
ros2_ws/
 └── src/
     └── nav2_config/
         ├── nav2_params.yaml
         └── README.md  (本文件)
```

### 2. Git 管理建议

```bash
git add nav2_params.yaml README.md
git commit -m "Add Nav2 tuned parameters for TurtleBot3"
```

---
当前配置状态：

* 稳定可用
* 定位可靠
* 导航成功率高
* 属于工程增强版

维护策略：

> 以稳定为主，小步试验，避免大改。



---

（完）
