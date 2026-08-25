#pragma once

#include <array>
#include <string>

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
  float update_rate_hz{100.F};
  bool prevent_arming{true};
  bool activate_even_while_disarmed{false};
  // Empty for a real vehicle ('/fmu/...'); e.g. "/drone0" for a namespaced simulator.
  // Retained only so the resolved value can be logged at startup.
  std::string topic_namespace_prefix;
};

struct ReferenceParameters {
  ReferenceSource position_source{ReferenceSource::Activation};
  std::array<float, 3> configured_position_ned_m{0.F, 0.F, -2.F};
  ReferenceSource yaw_source{ReferenceSource::Activation};
  float configured_yaw_ned_rad{0.F};
  float max_initial_offset_m{2.F};
};

struct SetpointParameters {
  std::array<bool, 3> position_enabled{false, false, false};
  std::array<bool, 3> velocity_enabled{true, true, true};
  std::array<bool, 3> acceleration_enabled{true, true, true};
  bool yaw_enabled{true};
  bool yaw_rate_enabled{false};

  std::array<float, 3> position_offset_ned_m{0.F, 0.F, 0.F};
  std::array<float, 3> velocity_base_ned_m_s{0.F, 0.F, 0.F};
  std::array<float, 3> acceleration_base_ned_m_s2{0.F, 0.F, 0.F};
  float yaw_offset_rad{0.F};
  float yaw_rate_base_rad_s{0.F};
};

struct SweepParameters {
  ExcitationTarget target{ExcitationTarget::AccelerationY};
  HorizontalFrame horizontal_frame{HorizontalFrame::Ned};
  Waveform waveform{Waveform::Linear};
  float start_frequency_hz{0.1F};
  float end_frequency_hz{5.F};
  float amplitude{0.5F};
  float duration_s{30.F};
  float phase_offset_rad{0.F};
  float fade_in_s{1.F};
  float fade_out_s{1.F};
  int repetitions{1};
  float settle_before_s{5.F};
  float settle_between_s{5.F};
  float settling_timeout_s{30.F};
  float position_tolerance_m{0.3F};
  float velocity_tolerance_m_s{0.25F};
  float minimum_samples_per_cycle{10.F};
};

struct VelocityIntegratorParameters {
  bool enabled{true};
  float start_delay_s{0.1F};
  float leak_time_constant_s{1.33F};
  float max_abs_velocity_m_s{3.F};
};

struct SafetyParameters {
  bool enabled{true};
  bool abort_on_violation{true};
  float max_horizontal_deviation_m{5.F};
  float max_vertical_deviation_m{2.F};
  float max_speed_m_s{5.F};
  float max_tilt_deg{35.F};
  float telemetry_timeout_s{0.5F};
};

struct LoggingParameters {
  bool enabled{true};
  bool required{false};
  std::string directory{"/tmp/px4_frequency_sweep"};
  int flush_every_n_samples{100};
};

struct FrequencySweepParameters {
  ModeParameters mode;
  ReferenceParameters reference;
  SetpointParameters setpoint;
  SweepParameters sweep;
  VelocityIntegratorParameters velocity_integrator;
  SafetyParameters safety;
  LoggingParameters logging;
};

}  // namespace px4_frequency_sweep
