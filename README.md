# PX4 Frequency Sweep

A configurable PX4 ROS 2 external flight mode for frequency-sweep experiments. The mode is built
on the official [`px4_ros2_interface_lib`](https://github.com/Auterion/px4-ros2-interface-lib)
and uses `TrajectorySetpointType`; it does not use MAVROS and does not switch PX4 into Offboard.

[中文说明](README.zh-CN.md)

> [!WARNING]
> Frequency sweeps deliberately excite vehicle dynamics. Validate the configuration in SITL,
> restrain the vehicle when appropriate, keep a pilot ready to switch modes, and start with a low
> amplitude and low maximum frequency. This repository has not been flight-validated on every
> airframe.

## What this package does

When the ROS 2 node starts, it dynamically registers a PX4 mode named **Frequency Sweep**. The
vehicle must already be armed, airborne, and hovering in another mode. After the pilot selects
Frequency Sweep, the mode:

1. Captures or loads a configurable NED position/yaw reference.
2. Commands a full position/yaw hold until the vehicle is stable.
3. Generates a mathematically integrated linear or logarithmic chirp.
4. Adds the chirp to one configurable trajectory component.
5. Optionally integrates acceleration excitation into a leaky velocity feed-forward.
6. Checks telemetry freshness, position deviation, velocity, and tilt limits.
7. Returns to a position/yaw hold and reports completion to PX4.

The mode never arms, takes off, lands, or selects another PX4 mode. Those actions belong to the
pilot/GCS or to a future `ModeExecutor`.

## Supported excitation targets

The `sweep.target` parameter exposes every field supported by the library's flexible trajectory
setpoint:

| Target | Amplitude unit | PX4 control layer |
|---|---:|---|
| `position_x`, `position_y`, `position_z` | m | Position setpoint |
| `velocity_x`, `velocity_y`, `velocity_z` | m/s | Velocity setpoint |
| `acceleration_x`, `acceleration_y`, `acceleration_z` | m/s² | Acceleration/feed-forward setpoint |
| `yaw` | rad | Yaw-angle setpoint |
| `yaw_rate` | rad/s | Yaw-rate setpoint |

Horizontal targets can use either:

- `sweep.horizontal_frame: "ned"`: X/Y are North/East.
- `sweep.horizontal_frame: "heading"`: X/Y are forward/right at activation yaw and are rotated
  into NED before publishing.

This is still a `TrajectorySetpoint` mode. It does not directly inject body-rate, torque, or motor
commands.

## Repository layout

```text
Px4FrequencySweep/
├── config/frequency_sweep.yaml       # all vehicle/experiment parameters
├── launch/frequency_sweep.launch.py  # config_file launch argument
├── include/px4_frequency_sweep/
├── src/
├── test/sweep_generator_test.cpp
├── CMakeLists.txt
└── package.xml
```

The upstream interface library is a dependency and is not copied or modified here.

## Compatibility

PX4, `px4_msgs`, and `px4_ros2_interface_lib` must use matching message definitions. Use all three
from `main`, or use matching `release/<PX4-version>` branches. Do not mix an arbitrary PX4 firmware
with a different `px4_msgs` branch.

This package uses the C++ APIs present in current `main` and the PX4 1.17 interface branch:

- `px4_ros2::ModeBase`
- `px4_ros2::NodeWithMode`
- `px4_ros2::TrajectorySetpointType`
- `px4_ros2::OdometryLocalPosition`
- `px4_ros2::OdometryAttitude`
- `px4_ros2::OdometryAngularVelocity`

## Build

Clone this repository beside the already-matched `px4_msgs` and interface library packages:

```text
~/drone_ros2_ws/src/
├── px4_msgs/
├── px4-ros2-interface-lib/
└── Px4FrequencySweep/
```

Then build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/drone_ros2_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to px4_frequency_sweep
source install/setup.bash
```

## Configure

Copy [`config/frequency_sweep.yaml`](config/frequency_sweep.yaml) to a vehicle-specific file and
edit that copy. Parameters are validated at startup; an invalid frequency range, array length, or
sample-rate combination prevents the node from registering.

### Horizontal velocity pairing

PX4 expects horizontal X/Y setpoint components to be valid as a pair. The default therefore uses:

```yaml
setpoint.velocity_enabled: [true, true, true]
```

During an `acceleration_y` sweep, Y receives the leaky integral of the excitation and X receives its
configured zero baseline. An `acceleration_x` sweep uses the opposite mapping. This preserves the
legacy intent of allowing motion on the excited axis while constraining drift on the orthogonal
axis, without producing a split finite/`NaN` horizontal pair.

Every position, velocity, and acceleration component has a separate enable flag. Disabled
components become `NaN`; they are not silently filled with zero. The selected sweep target is
always enabled because it is the experiment input.

When creating a vehicle-specific configuration, keep each horizontal X/Y term either valid as a
pair or `NaN` as a pair. The legacy pitch run wrote an integrated X velocity but masked both X and Y
velocity fields, so that written X value was not effective; the reusable default deliberately fixes
that inconsistency instead of reproducing it.

### Initial hover reference

The reusable default captures the actual hover point when the mode is selected:

```yaml
reference.position_source: "activation"
reference.yaw_source: "activation"
```

To use a fixed local reference:

```yaml
reference.position_source: "configured"
reference.configured_position_ned_m: [0.0, -5.0, -2.6]
reference.yaw_source: "configured"
reference.configured_yaw_ned_rad: 0.0
```

All values are PX4 local **NED**, not MAVROS ENU. NED Z is negative above the local origin. A
configured reference farther than `reference.max_initial_offset_m` from the activation position is
rejected.

### Sampling guard

The node requires:

```text
mode.update_rate_hz >= sweep.end_frequency_hz × sweep.minimum_samples_per_cycle
```

The default is 100 Hz, 5 Hz, and 10 samples/cycle. Increasing the ROS 2 publication rate does not
increase the internal PX4 controller rate, so verify the effective timestamps in ULog before using
a high maximum frequency.

### Acceleration-to-velocity integration

For acceleration targets, the optional leaky trapezoidal integrator recreates the intent of the
legacy ROS 1 implementation without hard-coding a sample-rate-dependent alpha:

```yaml
velocity_integrator.enabled: true
velocity_integrator.start_delay_s: 0.1
velocity_integrator.leak_time_constant_s: 1.33
velocity_integrator.max_abs_velocity_m_s: 3.0
```

Integrated velocity is published only on axes enabled by `setpoint.velocity_enabled`.

## Run in SITL

Start PX4 SITL and the XRCE-DDS agent:

```bash
# Terminal 1
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2
MicroXRCEAgent udp4 -p 8888
```

Launch the mode:

```bash
# Terminal 3
source ~/drone_ros2_ws/install/setup.bash
ros2 launch px4_frequency_sweep frequency_sweep.launch.py
```

To use an external configuration file:

```bash
ros2 launch px4_frequency_sweep frequency_sweep.launch.py \
  config_file:=/absolute/path/to/my_aircraft_sweep.yaml
```

Check registration in the PX4 shell:

```text
commander status
```

Take off in Position/Hold, stabilize, and then select **Frequency Sweep** through RC/QGC. Because
`mode.prevent_arming` is true by default, the vehicle cannot be armed while this mode is selected.

## Run on a companion computer

The PX4 `uxrce_dds_client` must connect to a `MicroXRCEAgent` process on the companion computer.
For example, for a serial link:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Then launch this package exactly as in SITL. A real vehicle normally uses the stock `/fmu/...`
topics, so `px4_topic_namespace_prefix` stays empty.

If PX4 is configured with a `UXRCE_DDS_NS` — common in simulators that expose `drone0/fmu/...` or
`iris1/fmu/...` — set that prefix in the YAML. Note this is *not* a ROS node namespace: a node
namespace does not move the `/fmu/...` topics at all.

```yaml
/**:
  ros__parameters:
    px4_topic_namespace_prefix: "/drone0"   # leading slash added automatically if omitted
```

To override it for a single run without editing the file:

```bash
ros2 run px4_frequency_sweep frequency_sweep_mode --ros-args \
  --params-file install/px4_frequency_sweep/share/px4_frequency_sweep/config/frequency_sweep.yaml \
  -p px4_topic_namespace_prefix:=/drone0
```

Confirm the resolved topics match what PX4 publishes:

```bash
ros2 node info /frequency_sweep_mode   # Subscribers show the fully resolved names
ros2 topic list | grep fmu
```

## Logging

When enabled, the mode writes one CSV file per activation. The file contains:

- ROS time, phase, repetition, chirp frequency/envelope/value.
- Every commanded trajectory component; disabled fields are `NaN`.
- Measured NED position/velocity, RPY attitude, and FRD angular velocity.
- Metadata comments with the target, frequency range, amplitude, and reference.

The default directory is `/tmp/px4_frequency_sweep`, which may be cleared on reboot. Configure an
absolute persistent path on the companion computer. PX4 ULog remains the preferred source for
internal controller setpoints and actuator data that are not exported by the default DDS topic
set.

## Differences from the legacy ROS 1 script

See [`docs/migration_from_ros1.md`](docs/migration_from_ros1.md) for the exact behavior mapping.
Important changes include dynamic PX4 mode registration, NED-native fields, configurable component
masks, correct chirp phase integration, rate-independent velocity leakage, and failure-to-hold
behavior. Offline frequency-response fitting is intentionally kept separate from the flight-mode
process.

## Tests

The chirp and velocity integrator have a ROS-independent test:

```bash
cd ~/drone_ros2_ws
colcon test --packages-select px4_frequency_sweep
colcon test-result --verbose
```

Always complete SITL tests before real-flight tests.
