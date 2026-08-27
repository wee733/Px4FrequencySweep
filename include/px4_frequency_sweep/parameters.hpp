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
  // Empty for a real vehicle; "/drone0" for a namespaced simulator.
  std::string topic_namespace_prefix;
};

struct ReferenceParameters {
  // Configured triggers a transit to configured_position_ned_m; Activation captures the
  // handover point and skips the transit.
  ReferenceSource position_source{ReferenceSource::Activation};
  std::array<float, 3> configured_position_ned_m{0.F, 0.F, -2.6F};
  // Configured by default: a captured yaw rotates the excitation axis by whatever heading the
  // pilot was holding.
  ReferenceSource yaw_source{ReferenceSource::Configured};
  float configured_yaw_ned_rad{0.F};
  // Transit budget, not a proximity requirement. Guards against a mistyped reference or a
  // stale EKF origin.
  float max_transit_distance_m{30.F};
  float transit_timeout_s{60.F};
};

// One excitation stage. The setpoint mask travels with the stage because each axis needs a
// different combination of published components.
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
  // an axis can hold a fixed baseline without receiving the integral.
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
  float minimum_samples_per_cycle{7.5F};
};

struct VelocityIntegratorParameters {
  bool enabled{true};
  float start_delay_s{0.1F};
  // tau = -dt/ln(alpha); 1.33 s corresponds to alpha=0.995 at 150 Hz.
  float leak_time_constant_s{1.33F};
  float max_abs_velocity_m_s{3.F};
};

struct SafetyParameters {
  bool enabled{true};
  bool abort_on_violation{true};
  // Only the altitude band is enforced by default; tighten the rest once the excitation
  // envelope is known from a flown run.
  float max_horizontal_deviation_m{1000.F};
  float max_vertical_deviation_m{2.1F};
  float max_speed_m_s{1000.F};
  float max_tilt_deg{89.F};
  float telemetry_timeout_s{0.5F};
};

struct LoggingParameters {
  bool enabled{true};
  bool required{false};
  // Relative to the node's working directory; use an absolute path on a companion computer.
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
