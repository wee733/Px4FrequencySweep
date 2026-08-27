# PX4 Frequency Sweep

A configurable PX4 ROS 2 external flight mode for frequency-sweep experiments. The mode is built
on the official [`px4_ros2_interface_lib`](https://github.com/Auterion/px4-ros2-interface-lib)
and uses `TrajectorySetpointType`.

it based on https://github.com/xuhao1/pyAircraftIden.git

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

Each stage's `target` selects one field of the trajectory setpoint:

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

### Stages

`sweep.sequence` names the stages to fly, in order. Each stage carries its own excitation target,
amplitude, and setpoint profile, because the legacy ROS 1 script used a different `type_mask` per
axis rather than one shared profile:

```yaml
sweep.sequence: ["roll", "pitch", "yaw"]

stages.roll.target: "acceleration_y"
stages.roll.amplitude: 3.0
stages.roll.velocity_enabled: [true, true, true]
stages.roll.integrator_publish_enabled: [false, true, false]
```

`sweep.repetitions` applies per stage, so the shipped default flies 3 axes × 3 repetitions in one
activation. Every stage writes to the same CSV; the `stage` and `target` columns separate them.

`velocity_enabled` decides whether an axis is velocity-controlled at all;
`integrator_publish_enabled` decides which axes carry the acceleration integral. Keeping these
separate is what lets the roll stage pin `VX` to 0 while `VY` follows the excitation integral.

### Horizontal pairing

PX4's `PositionControl::_inputValid()` rejects the **entire** `TrajectorySetpoint` unless the X and
Y components of each of position, velocity, and acceleration are either both finite or both `NaN`,
and unless every axis carries at least one finite setpoint. A split pair does not degrade
gracefully — the setpoint is dropped and PX4 falls back to its own failsafe.

Startup validation enforces both rules per stage, so a bad profile is a launch error rather than a
mid-flight failsafe. All three shipped stages keep their pairs intact: roll has both horizontal
velocities finite, pitch and yaw have both unset.

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

The ROS 1-aligned default is 150 Hz, 20 Hz, and 7.5 samples/cycle — exactly at the limit, which is
what 150 Hz over a 0.1–20 Hz band gives.

Raising the ROS 2 publication rate does not raise PX4's internal controller rate, so the setpoint
PX4 actually applies is resampled at whatever rate `mc_pos_control` runs. Take the excitation and
response from the ULog rather than from this node's CSV, so both sit on PX4's own clock.

### Acceleration-to-velocity integration

For acceleration targets, the optional leaky trapezoidal integrator recreates the intent of the
legacy ROS 1 implementation without hard-coding a sample-rate-dependent alpha:

```yaml
velocity_integrator.enabled: true
velocity_integrator.start_delay_s: 0.1
velocity_integrator.leak_time_constant_s: 1.33
velocity_integrator.max_abs_velocity_m_s: 3.0
```

`leak_time_constant_s: 1.33` is the ROS 1 leak expressed rate-independently: that script applied
`alpha = 0.995` at 150 Hz, i.e. `tau = -dt/ln(alpha) = 1.33 s`. Its comment claimed a 0.008 Hz
corner, but the real corner is `1/(2*pi*tau) = 0.12 Hz` — above the 0.1 Hz start frequency, so the
integrator attenuates and phase-shifts the bottom of the sweep. That is inherited behaviour, not a
new choice.

The integral reaches only axes where **both** `velocity_enabled` and `integrator_publish_enabled`
are set for that axis.

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

The shipped configuration reproduces the legacy experiment: three axes in one flight, 3 repetitions
each, 0.1–20 Hz over 60 s at 150 Hz, amplitude 3.0 m/s² (0.9 rad on yaw), no fade, yaw locked to 0,
and the same per-axis setpoint profiles.

**One deliberate difference.** The ROS 1 chirp computed its phase as `sin(2π·f(t)·t)` with `f(t)`
ramping linearly. That multiplies the ramped frequency by `t` instead of integrating it, so the
instantaneous frequency was `f_min + 2(f_max−f_min)·t/T` — the sweep actually reached **39.9 Hz**,
twice its nominal 20 Hz, and the frequency logged alongside each sample was wrong. This is a defect,
not a design choice, so the phase here is integrated properly and the sweep really ends at 20 Hz.
Archived ROS 1 data is therefore not sample-for-sample comparable with a ROS 2 run.

Structural changes: dynamic PX4 mode registration, NED-native fields, per-stage setpoint profiles,
startup validation of PX4's horizontal pairing rule, rate-independent velocity leakage, and
failure-to-hold behaviour. Offline frequency-response fitting stays outside the flight-mode process.

See [`docs/migration_from_ros1.md`](docs/migration_from_ros1.md) for the full mapping.

## Tests

The chirp and velocity integrator have a ROS-independent test:

```bash
cd ~/drone_ros2_ws
colcon test --packages-select px4_frequency_sweep
colcon test-result --verbose
```

Always complete SITL tests before real-flight tests.
