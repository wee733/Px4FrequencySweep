#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace px4_frequency_sweep {

enum class ModePhase {
  Inactive,
  // Flying to a configured reference that is deliberately away from the activation point. Kept
  // separate from settling so the safety deviation budget can account for the transit: measured
  // from the reference, the aircraft starts a full transit distance away, which would otherwise
  // trip max_horizontal_deviation_m the instant the mode activates.
  TransitToReference,
  SettlingBeforeSweep,
  Sweeping,
  SettlingBetweenSweeps,
  CompletedHold,
  FailureHold,
};

inline constexpr std::string_view toString(ModePhase phase)
{
  switch (phase) {
    case ModePhase::Inactive:
      return "inactive";
    case ModePhase::TransitToReference:
      return "transit_to_reference";
    case ModePhase::SettlingBeforeSweep:
      return "settling_before_sweep";
    case ModePhase::Sweeping:
      return "sweeping";
    case ModePhase::SettlingBetweenSweeps:
      return "settling_between_sweeps";
    case ModePhase::CompletedHold:
      return "completed_hold";
    case ModePhase::FailureHold:
      return "failure_hold";
  }
  return "unknown";
}

struct TrajectoryCommand {
  std::array<std::optional<float>, 3> position_ned_m{};
  std::array<std::optional<float>, 3> velocity_ned_m_s{};
  std::array<std::optional<float>, 3> acceleration_ned_m_s2{};
  std::optional<float> yaw_ned_rad;
  std::optional<float> yaw_rate_ned_rad_s;
};

struct TelemetrySnapshot {
  std::array<float, 3> position_ned_m{};
  std::array<float, 3> velocity_ned_m_s{};
  std::array<float, 3> attitude_rpy_rad{};
  std::array<float, 3> angular_velocity_frd_rad_s{};

  // PX4-clock timestamps carried by the angular velocity message (microseconds since PX4 boot).
  // These are what makes a CSV row addressable in the ulog: 'ros_time_s' is the companion
  // computer's clock, and nothing else in the file relates the two time bases. 'publish' is when
  // PX4 sent the message, 'sample' is when the underlying data was measured -- the gap between
  // 'sample' and the ROS receive time is the offboard pipeline delay.
  // Zero means the angular velocity subscription had no valid message for this row.
  uint64_t px4_timestamp_us{0};
  uint64_t px4_timestamp_sample_us{0};
};

}  // namespace px4_frequency_sweep
