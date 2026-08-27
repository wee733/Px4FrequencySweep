#pragma once

#include <array>
#include <string>
#include <vector>

namespace px4_frequency_sweep {

enum class Waveform { Linear, Logarithmic };
enum class ReferenceSource { Activation, Configured };
enum class HorizontalFrame { Ned, Heading };

enum class ExcitationTarget {
  PositionX,
  PositionY,
  PositionZ,
  VelocityX,
  VelocityY,
  VelocityZ,
  AccelerationX,
  AccelerationY,
  AccelerationZ,
  Yaw,
  YawRate,
};

struct ModeParameters {
  std::string name{"Frequency Sweep"};
  float update_rate_hz{150.F};
  bool prevent_arming{true};
  bool activate_even_while_disarmed{false};
  // Empty for a real vehicle ('/fmu/...'); e.g. "/drone0" for a namespaced simulator.
  // Retained only so the resolved value can be logged at startup.
  std::string topic_namespace_prefix;
};

struct ReferenceParameters {
  // Configured: the mode flies to configured_position_ned_m on activation and treats it as the
  // origin for every offset and safety limit. Activation: whatever point the pilot hands over at
  // becomes the origin, and no transit happens.
  ReferenceSource position_source{ReferenceSource::Activation};
  std::array<float, 3> configured_position_ned_m{0.F, 0.F, -2.6F};
  // The legacy ROS 1 script commanded an absolute yaw of 0 for every run, which is what made
  // its NED-Y excitation coincide with body-right. Capturing yaw at activation instead silently
  // rotates the excitation axis by whatever heading the pilot happened to be holding.
  ReferenceSource yaw_source{ReferenceSource::Configured};
  float configured_yaw_ned_rad{0.F};
  // How far the mode may fly to reach a configured reference. This is a transit budget, not a
  // requirement to already be there: exceeding it aborts, on the assumption that a reference tens
  // of metres from the aircraft is a mistyped coordinate or a stale EKF origin.
  float max_transit_distance_m{30.F};
  float transit_timeout_s{60.F};
};

// One excitation stage. The setpoint profile travels with the stage because the legacy ROS 1
// script used a different type_mask per axis: its roll run commanded VX=0 and VY=integrated,
// while its pitch and yaw runs left both horizontal velocities unset.
struct SweepStage {
  std::string name{"stage"};
  ExcitationTarget target{ExcitationTarget::AccelerationY};
  float amplitude{3.F};

  std::array<bool, 3> position_enabled{false, false, false};
  std::array<bool, 3> velocity_enabled{true, true, true};
  std::array<bool, 3> acceleration_enabled{true, true, true};
  bool yaw_enabled{true};
  bool yaw_rate_enabled{true};

  std::array<float, 3> position_offset_ned_m{0.F, 0.F, 0.F};
  std::array<float, 3> velocity_base_ned_m_s{0.F, 0.F, 0.F};
  std::array<float, 3> acceleration_base_ned_m_s2{0.F, 0.F, 0.F};
  float yaw_offset_rad{0.F};
  float yaw_rate_base_rad_s{0.F};

  // Which velocity components carry the acceleration integral. Distinct from velocity_enabled:
  // an axis can be held at a fixed velocity baseline without receiving the excitation integral,
  // which is what the ROS 1 roll run did (VX pinned to 0, integral on VY only).
  std::array<bool, 3> integrator_publish_enabled{false, true, false};
};

struct SweepParameters {
  HorizontalFrame horizontal_frame{HorizontalFrame::Ned};
  Waveform waveform{Waveform::Linear};
  float start_frequency_hz{0.1F};
  float end_frequency_hz{20.F};
  float duration_s{60.F};
  float phase_offset_rad{0.F};
  float fade_in_s{0.F};
  float fade_out_s{0.F};
  int repetitions{3};
  float settle_before_s{10.F};
  float settle_between_s{20.F};
  float settling_timeout_s{60.F};
  float position_tolerance_m{0.2F};
  float velocity_tolerance_m_s{0.25F};
  // 150 Hz / 20 Hz = 7.5 samples per cycle at the top of the band.
  float minimum_samples_per_cycle{7.5F};
};

struct VelocityIntegratorParameters {
  bool enabled{true};
  float start_delay_s{0.1F};
  // The ROS 1 leak was alpha=0.995 applied at 150 Hz, i.e. tau = -dt/ln(alpha) = 1.33 s.
  float leak_time_constant_s{1.33F};
  float max_abs_velocity_m_s{3.F};
};

struct SafetyParameters {
  bool enabled{true};
  bool abort_on_violation{true};
  // The ROS 1 script had every check except the altitude band commented out. These defaults
  // reproduce that: the altitude limit is real, the others are effectively disabled. Tighten
  // them once a run has been flown and the excitation envelope is known.
  float max_horizontal_deviation_m{1000.F};
  // ROS 1 aborted outside 0.5..5.0 m absolute with a 2.6 m reference, i.e. -2.1/+2.4 m. This
  // symmetric limit takes the tighter side.
  float max_vertical_deviation_m{2.1F};
  float max_speed_m_s{1000.F};
  float max_tilt_deg{89.F};
  float telemetry_timeout_s{0.5F};
};

struct LoggingParameters {
  bool enabled{true};
  bool required{false};
  // Resolved against the working directory the node is launched from, matching the YAML default.
  // Deliberately not /tmp, which is cleared on reboot. Override with an absolute path on a
  // companion computer.
  std::string directory{"logs"};
  int flush_every_n_samples{100};
};

struct FrequencySweepParameters {
  ModeParameters mode;
  ReferenceParameters reference;
  SweepParameters sweep;
  std::vector<SweepStage> stages;
  VelocityIntegratorParameters velocity_integrator;
  SafetyParameters safety;
  LoggingParameters logging;
};

}  // namespace px4_frequency_sweep
