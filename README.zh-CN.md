# PX4 Frequency Sweep（中文说明）

这是一个基于官方 `px4_ros2_interface_lib` 的 PX4 ROS 2 外部飞行模式。它使用
`TrajectorySetpointType` 完成扫频实验，不依赖 MAVROS，也不会切换到 Offboard。

## 设计边界

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

完整配置见 [`config/frequency_sweep.yaml`](config/frequency_sweep.yaml)。建议复制一份作为
具体无人机的配置，不要直接把不同飞机的参数混在同一个文件中。

### 扫什么

```yaml
sweep.target: "acceleration_y"
```

可选值：

```text
position_x/y/z
velocity_x/y/z
acceleration_x/y/z
yaw
yaw_rate
```

幅值单位随 target 改变，分别是 m、m/s、m/s²、rad、rad/s。

水平 X/Y 默认是 NED 的 North/East：

```yaml
sweep.horizontal_frame: "ned"
```

若希望 X/Y 表示激活时机头的前向/右向，可设置：

```yaml
sweep.horizontal_frame: "heading"
```

代码会使用激活时 yaw 把水平激励旋转到 NED。

### 水平速度约束

默认配置是：

```yaml
setpoint.velocity_enabled: [true, true, true]
```

PX4 要求水平 X/Y 设定值成对有效。扫 `acceleration_y` 时，Y 速度使用加速度的泄漏积分，
X 速度使用配置的零基线来限制漂移；扫 `acceleration_x` 时反过来。这样保留了 ROS 1
代码“激励轴允许运动、正交轴命令零速度”的意图，同时不会产生一轴有限、一轴 `NaN` 的非法组合。

位置、速度、加速度的每个基线/辅助分量都能分别开关：

```yaml
setpoint.position_enabled: [false, false, false]
setpoint.velocity_enabled: [true, true, true]
setpoint.acceleration_enabled: [true, true, true]
setpoint.yaw_enabled: true
setpoint.yaw_rate_enabled: false
```

被选为 `sweep.target` 的分量会始终发送，因为它就是实验输入。例如即使
`setpoint.velocity_enabled[0]` 为 `false`，选择 `velocity_x` 扫频时仍会发送 X 速度扫频。
自定义配置时仍应确保所有水平 X/Y 项成对有效或成对为 `NaN`。

### 初始化位置

推荐在选择模式时捕获当前悬停点：

```yaml
reference.position_source: "activation"
reference.yaw_source: "activation"
```

也可以使用固定参考：

```yaml
reference.position_source: "configured"
reference.configured_position_ned_m: [0.0, -5.0, -2.6]
reference.yaw_source: "configured"
reference.configured_yaw_ned_rad: 0.0
```

这里全部是 PX4 本地 NED，不是 MAVROS ENU。NED 中向上飞时 Z 为负。

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

CSV 默认写入 `/tmp/px4_frequency_sweep`。真机请改成机载电脑上的持久化绝对路径。CSV 记录
外部指令和基础状态；PX4 内部角速度设定值、控制器输出和电机数据仍建议使用 ULog。

> 真机前必须先做 SITL；第一次真机实验应使用低幅值、低最高频率、单轴、单次扫频，并保证
> 飞手可随时通过 RC 夺回模式控制权。
