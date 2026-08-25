#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include "px4_frequency_sweep/configuration.hpp"
#include "px4_frequency_sweep/csv_logger.hpp"
#include "px4_frequency_sweep/sweep_generator.hpp"
#include "px4_frequency_sweep/types.hpp"

namespace px4_frequency_sweep {

class FrequencySweepMode : public px4_ros2::ModeBase {
 public:
  explicit FrequencySweepMode(rclcpp::Node& node);

  void checkArmingAndRunConditions(
      px4_ros2::HealthAndArmingCheckReporter& reporter) override;
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

 private:
  void transitionTo(ModePhase phase);
  void startSweep();
  void finishSuccessfully();
  void enterFailureHold(const std::string& reason);

  bool telemetryValid() const;
  bool referenceReached() const;
  std::optional<std::string> safetyViolation() const;

  TrajectoryCommand holdCommand(const std::array<float, 3>& position_ned_m,
                                float yaw_ned_rad) const;
  TrajectoryCommand sweepCommand(const SweepSample& sample, float dt_s,
                                 float sweep_elapsed_s);
  void publish(const TrajectoryCommand& command);
  void publishAndLog(const TrajectoryCommand& command, const SweepSample& sample,
                     float phase_elapsed_s);
  TelemetrySnapshot telemetrySnapshot() const;

  static void setComponent(std::array<std::optional<float>, 3>& values, std::size_t axis,
                           float value);
  static float wrapPi(float angle_rad);

  FrequencySweepParameters parameters_;
  SweepGenerator sweep_generator_;
  LeakyVelocityIntegrator velocity_integrator_;

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> local_position_;
  std::shared_ptr<px4_ros2::OdometryAttitude> attitude_;
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> angular_velocity_;

  CsvLogger csv_logger_;
  ModePhase phase_{ModePhase::Inactive};
  rclcpp::Time phase_start_time_{};
  rclcpp::Time last_safety_warning_time_{};
  float stable_time_s_{0.F};
  int repetition_index_{0};
  bool completion_reported_{false};
  bool reference_initialized_{false};

  std::array<float, 3> reference_position_ned_m_{};
  float reference_yaw_ned_rad_{0.F};
  std::array<float, 3> failure_hold_position_ned_m_{};
  float failure_hold_yaw_ned_rad_{0.F};
};

}  // namespace px4_frequency_sweep
