# Nav2 性能稳定性三项修改说明

本文档记录在 TurtleBot3 + Nav2 + Gazebo 环境中，为解决 **CPU 过载、TF 时间戳异常、规划掉帧** 等问题而进行的三项最小侵入式优化修改。

原则：

* 不修改导航算法逻辑
* 不影响定位 / 建图 / 避障功能
* 仅降低系统负载、提高稳定性
* 可随时回滚

---

## 一、修改背景

原系统在运行中出现：

* TF 报错：timestamp earlier than transform cache
* Planner 频率下降（< 5Hz）
* RViz 丢帧
* CPU 占用过高

根因：

* DWB 调试日志过多
* Waypoint 循环频率异常
* Global Costmap 3D 体素计算负载过高

---

## 二、修改项总览

| 编号 | 模块             | 修改项            | 目的        |
| -- | -------------- | -------------- | --------- |
| 1  | Controller     | 关闭 DWB Debug   | 减少日志开销    |
| 2  | Waypoint       | 降低循环频率         | 降低 CPU 占用 |
| 3  | Global Costmap | 移除 voxel layer | 降低规划负载    |

---

## 三、修改项详细说明

---

### 1. 关闭 DWB 轨迹调试日志

#### 原配置

```yaml
FollowPath:
  debug_trajectory_details: True
```

#### 修改后

```yaml
FollowPath:
  debug_trajectory_details: False
```

#### 说明

* 仅关闭轨迹评分日志输出
* 不影响控制算法
* 不影响轨迹生成
* 可运行时动态开启

动态开启方式：

```bash
ros2 param set /controller_server FollowPath.debug_trajectory_details true
```

---

### 2. 降低 Waypoint Follower 运行频率

#### 原配置

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 2000
```

#### 修改后

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 50
```

#### 说明

* 原 2000Hz 对实际控制无收益
* 导致无效 CPU 消耗
* 50Hz 已满足控制需求
* 不影响多点导航功能

---

### 3. 移除 Global Costmap 中的 Voxel Layer

#### 原配置

```yaml
plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
```

#### 修改后

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

并删除 global_costmap 下对应 voxel_layer 配置块。

#### 说明

* Global costmap 主要用于全局路径规划
* TurtleBot3 使用 2D 雷达
* Voxel layer 在 global 中收益有限
* 保留 local voxel 不影响避障

结构变化：

修改前：

```
Global: static + obstacle + voxel + inflation
Local : obstacle + voxel + inflation
```

修改后：

```
Global: static + obstacle + inflation
Local : obstacle + voxel + inflation
```

---

## 四、修改影响评估

| 项目         | 修改前 | 修改后 |
| ---------- | --- | --- |
| CPU 使用率    | 偏高  | 稳定  |
| TF 异常      | 偶发  | 极少  |
| Planner 频率 | 不稳定 | 稳定  |
| 定位功能       | 正常  | 正常  |
| 避障能力       | 正常  | 正常  |
| 导航成功率      | 波动  | 稳定  |

结论：

修改不会破坏核心功能，仅提升系统稳定性。

---

## 五、回滚方式

修改前建议备份：

```bash
cp nav2_params.yaml nav2_params.yaml.bak
```

回滚：

```bash
mv nav2_params.yaml.bak nav2_params.yaml
```

---

## 六、验证方法

修改完成后验证：

```bash
ros2 topic hz /tf
ros2 topic hz /odom
ros2 topic hz /scan
```



