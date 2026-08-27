# PX4 Frequency Sweep（中文说明）

这是一个基于官方 `px4_ros2_interface_lib` 的 PX4 ROS 2 外部飞行模式。它使用
`TrajectorySetpointType` 完成扫频实验。

节点启动后会向 PX4 动态注册 **Frequency Sweep**。无人机应先在 Position/Hold 中完成
起飞和悬停，再由飞手通过 RC 或 QGC 选择该模式。此模式不负责解锁、起飞、降落或切换其他
模式。

执行流程：

```text
选择 Frequency Sweep
        ↓
读取激活位置或配置位置
        ↓
全位置和 yaw 保持，等待稳定
        ↓
执行一次或多次扫频
        ↓
回到参考点保持并报告完成
        ↓
飞手切换 Position/Hold/RTL/Land
```

## 最重要的配置

完整配置见 [`config/frequency_sweep.yaml`](config/frequency_sweep.yaml)。


### 参考点

用固定参考点（默认）。切入模式后飞机会**先飞到**这个点（阶段 `transit_to_reference`），到位并稳定后才开始激励：

```yaml
reference.position_source: "configured"
reference.configured_position_ned_m: [5.0, 0.0, -2.6]
reference.max_transit_distance_m: 30.0
reference.transit_timeout_s: 60.0
```

这个点随后成为**原点** —— 所有 stage 偏移量和所有安全偏差限值都以它为基准。这样不管飞手在哪里交接，每次扫频都从同一个绝对位置开始，多架次之间几何一致。

`max_transit_distance_m` 是"允许飞多远"的预算，不是"必须已经在那"。超了会 abort —— 假定坐标打错了或 EKF 原点是旧的。

也可以用交接点当参考：

```yaml
reference.position_source: "activation"
```

这时不发生转场，`configured_position_ned_m` 被完全忽略。

偏航建议始终用 `configured` + `0.0`：ROS1 就是这么做的，也正因如此它的 NED-Y 激励等于机体右侧。捕获偏航会让激励轴跟着飞手当时的机头朝向转 —— 在 1.72 rad 朝向下激活，名义上的 roll 扫频有 98.9% 打在 pitch 上。

```yaml
reference.yaw_source: "configured"
reference.configured_yaw_ned_rad: 0.0
```

这里全部是 PX4 本地 NED，不是 MAVROS ENU：Z 向上为负，X/Y 是北/东而非机体前/右。而且是**绝对**坐标，相对 EKF 原点 —— 换个起飞点，同样的数字就是另一个地方。

安全偏差是相对参考点算的，所以转场刚开始时飞机本来就差着整个转场距离。`transit_to_reference` 期间这段距离会加进水平/垂直限值，到位后移除 —— 否则参考点只要比 `safety.max_horizontal_deviation_m` 远，模式一激活就会 abort。

## 编译和运行

把仓库放在已有工作区中：

```text
~/drone_ros2_ws/src/
├── px4_msgs/
├── px4-ros2-interface-lib/
└── Px4FrequencySweep/
```

三者必须和 PX4 固件使用匹配的 main 或 `release/<version>` 分支。

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/drone_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to px4_frequency_sweep
source install/setup.bash
```

运行：

```bash
ros2 launch px4_frequency_sweep frequency_sweep.launch.py
```

指定外部配置：

```bash
ros2 launch px4_frequency_sweep frequency_sweep.launch.py \
  config_file:=/绝对路径/my_aircraft_sweep.yaml
```

适配带命名空间的仿真器：

PX4 话题前缀由 YAML 里的 `px4_topic_namespace_prefix` 决定，**不是** ROS 节点命名空间——
后者不会改变 `/fmu/...` 的解析结果。真机留空即可；仿真器如果配了 `UXRCE_DDS_NS`，
把它填进去（前导斜杠可省，会自动补上）：

```yaml
/**:
  ros__parameters:
    px4_topic_namespace_prefix: "/drone0"
```

临时覆盖，不改文件：

```bash
ros2 run px4_frequency_sweep frequency_sweep_mode --ros-args \
  --params-file install/px4_frequency_sweep/share/px4_frequency_sweep/config/frequency_sweep.yaml \
  -p px4_topic_namespace_prefix:=/drone0
```

确认解析结果和 PX4 实际发布的话题一致：

```bash
ros2 node info /frequency_sweep_mode   # 看 Subscribers 里的完整话题名
ros2 topic list | grep fmu
```

## 安全和数据

默认会检查：

- PX4 本地位置、速度和姿态消息是否超时；
- 相对参考点的水平和垂直偏移；
- 最大速度；
- 最大 roll/pitch 倾角。

越界后停止激励，捕获当前位置作为保持点，并向 PX4 报告模式失败。飞手仍应立即切换到合适的
PX4 模式。

CSV 默认写入 `logs`，相对于节点启动时的工作目录（不要用 `/tmp`，重启会被清空）。真机上请在
启动时覆盖成持久化绝对路径：

```bash
ros2 launch px4_frequency_sweep frequency_sweep.launch.py --ros-args \
  -p logging.directory:=/home/<user>/sweep_logs
```

辨识数据源仍是 ULog：`vehicle_rates_setpoint` → `vehicle_angular_velocity` 在控制器内部按 PX4
时钟采样，而 CSV 里的遥测列只是这些 ULog topic 经 DDS 轮询后的副本。CSV 不可替代的是两点：

1. 实验分段（`stage`/`repetition`/`phase`）。靠幅值阈值反推不出来——settling 阶段仍在发保持
   指令，yaw 级又被 `MPC_YAWRAUTO_MAX` 限幅成削顶三角波。
2. offboard 链路延迟。`ros_time_s` 减 `px4_timestamp_sample_us` 就是 ROS→PX4 的实际延迟分布，
   ULog 看不到这个量（它只知道 setpoint 何时进入 uORB，不知道 ROS 何时发出）。这个分布是
   sim2real 域随机化要填的数，也决定了 CSV 遥测列到多高频率还可信。

这两点都依赖 `px4_timestamp_us` / `px4_timestamp_sample_us` 两列：只有 `ros_time_s` 的话，CSV
和 ULog 之间没有任何共同时间基准，分段标记无法映射到 ULog。

角速度设定值、控制器输出和电机数据仍然只在 ULog 里，不从 DDS 抄。

> 真机前必须先做 SITL；第一次真机实验应使用低幅值、低最高频率、单轴、单次扫频，并保证
> 飞手可随时通过 RC 夺回模式控制权。
