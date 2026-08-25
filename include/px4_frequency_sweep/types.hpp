#pragma once

#include <array>
#include <optional>
#include <string_view>

namespace px4_frequency_sweep {

enum class ModePhase {
  Inactive,
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
};

}  // namespace px4_frequency_sweep
