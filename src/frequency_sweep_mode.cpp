#include "px4_frequency_sweep/frequency_sweep_mode.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <px4_ros2/components/events.hpp>

namespace px4_frequency_sweep {
namespace {

constexpr float kPi = 3.14159265358979323846F;

std::array<float, 3> toArray(const Eigen::Vector3f& vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

std::chrono::milliseconds telemetryTimeout(const FrequencySweepParameters& parameters)
{
  return std::chrono::milliseconds{
      static_cast<int64_t>(parameters.safety.telemetry_timeout_s * 1000.F)};
}

bool isHorizontalTarget(ExcitationTarget target)
{
  return target == ExcitationTarget::PositionX || target == ExcitationTarget::PositionY ||
         target == ExcitationTarget::VelocityX || target == ExcitationTarget::VelocityY ||
         target == ExcitationTarget::AccelerationX || target == ExcitationTarget::AccelerationY;
}

std::array<float, 2> horizontalDirectionNed(ExcitationTarget target, HorizontalFrame frame,
                                            float reference_yaw_ned_rad)
{
  const std::size_t axis = targetAxis(target);
  if (frame == HorizontalFrame::Ned) {
    return axis == 0 ? std::array<float, 2>{1.F, 0.F}
                     : std::array<float, 2>{0.F, 1.F};
  }

  const float cosine = std::cos(reference_yaw_ned_rad);
  const float sine = std::sin(reference_yaw_ned_rad);
  // Heading-frame X is forward; Y is right. Both are rotated into earth-fixed NED.
  return axis == 0 ? std::array<float, 2>{cosine, sine}
                   : std::array<float, 2>{-sine, cosine};
}

}  // namespace

// Initialisation order matters: the ModeBase argument expressions run first and declare
// 'mode.*' plus 'px4_topic_namespace_prefix', which declareAndLoadParameters() then reads back.
FrequencySweepMode::FrequencySweepMode(rclcpp::Node& node)
    : ModeBase(node, declareModeSettings(node), declareTopicNamespacePrefix(node)),
      parameters_(declareAndLoadParameters(node)),
      sweep_generator_(parameters_.sweep),
      velocity_integrator_(parameters_.velocity_integrator)
{
  trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
  angular_velocity_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
  setSetpointUpdateRate(parameters_.mode.update_rate_hz);

  RCLCPP_INFO(node.get_logger(),
              "Configured '%s': waveform=%s, %.3f..%.3f Hz, duration=%.1f s, "
              "repetitions=%d per stage, %zu stage(s), update_rate=%.1f Hz",
              parameters_.mode.name.c_str(), toString(parameters_.sweep.waveform).c_str(),
              parameters_.sweep.start_frequency_hz, parameters_.sweep.end_frequency_hz,
              parameters_.sweep.duration_s, parameters_.sweep.repetitions,
              parameters_.stages.size(), parameters_.mode.update_rate_hz);

  for (std::size_t index = 0; index < parameters_.stages.size(); ++index) {
    const SweepStage& stage = parameters_.stages[index];
    RCLCPP_INFO(node.get_logger(), "  stage %zu '%s': target=%s, amplitude=%.3f", index + 1,
                stage.name.c_str(), toString(stage.target).c_str(), stage.amplitude);
    const bool any_publish = stage.integrator_publish_enabled[0] ||
                             stage.integrator_publish_enabled[1] ||
                             stage.integrator_publish_enabled[2];
    if (any_publish && !isAccelerationTarget(stage.target)) {
      RCLCPP_WARN(node.get_logger(),
                  "    integrator_publish_enabled is set but target '%s' is not an acceleration "
                  "component, so the integral stays zero",
                  toString(stage.target).c_str());
    }
  }

  const float per_stage_s = static_cast<float>(parameters_.sweep.repetitions) *
                                (parameters_.sweep.duration_s + parameters_.sweep.settle_between_s) +
                            parameters_.sweep.settle_before_s;
  RCLCPP_INFO(node.get_logger(),
              "Estimated airborne time excluding settling overruns: %.0f s",
              static_cast<float>(parameters_.stages.size()) * per_stage_s);
  RCLCPP_INFO(node.get_logger(), "PX4 topics resolved under '%s/fmu/...'",
              parameters_.mode.topic_namespace_prefix.c_str());
}

void FrequencySweepMode::checkArmingAndRunConditions(
    px4_ros2::HealthAndArmingCheckReporter& reporter)
{
  if (!local_position_ || !attitude_) {
    return;
  }

  const auto timeout = telemetryTimeout(parameters_);
  const bool local_position_valid =
      local_position_->lastValid(timeout) && local_position_->last().xy_valid &&
      local_position_->last().z_valid && local_position_->last().v_xy_valid &&
      local_position_->last().v_z_valid;
  if (!local_position_valid) {
    reporter.armingCheckFailureExt(
        px4_ros2::events::ID("frequency_sweep_local_position_invalid"),
        px4_ros2::events::Log::Error, "Frequency Sweep requires valid local position");
  }

  if (!attitude_->lastValid(timeout)) {
    reporter.armingCheckFailureExt(px4_ros2::events::ID("frequency_sweep_attitude_invalid"),
                                   px4_ros2::events::Log::Error,
                                   "Frequency Sweep requires valid attitude");
  }
}

void FrequencySweepMode::onActivate()
{
  completion_reported_ = false;
  stage_index_ = 0;
  repetition_index_ = 0;
  stable_time_s_ = 0.F;
  velocity_integrator_.reset();
  reference_initialized_ = false;

  if (!telemetryValid()) {
    enterFailureHold("Cannot activate: local position or attitude telemetry is invalid");
    return;
  }

  const Eigen::Vector3f current_position = local_position_->positionNed();
  const float current_yaw = attitude_->yaw();
  failure_hold_position_ned_m_ = toArray(current_position);
  failure_hold_yaw_ned_rad_ = current_yaw;

  reference_position_ned_m_ =
      parameters_.reference.position_source == ReferenceSource::Activation
          ? toArray(current_position)
          : parameters_.reference.configured_position_ned_m;
  reference_yaw_ned_rad_ =
      parameters_.reference.yaw_source == ReferenceSource::Activation
          ? current_yaw
          : parameters_.reference.configured_yaw_ned_rad;
  reference_initialized_ = true;

  const Eigen::Vector3f configured_reference{reference_position_ned_m_[0],
                                              reference_position_ned_m_[1],
                                              reference_position_ned_m_[2]};
  const float reference_offset_m = (configured_reference - current_position).norm();
  if (reference_offset_m > parameters_.reference.max_initial_offset_m) {
    enterFailureHold("Configured reference is too far from the activation position (" +
                     std::to_string(reference_offset_m) + " m)");
    return;
  }

  if (parameters_.logging.enabled) {
    std::string error;
    if (!csv_logger_.open(parameters_, reference_position_ned_m_, reference_yaw_ned_rad_, error)) {
      if (parameters_.logging.required) {
        enterFailureHold("Required CSV logger could not start: " + error);
        return;
      }
      RCLCPP_WARN(node().get_logger(), "CSV logging disabled for this run: %s", error.c_str());
    } else {
      RCLCPP_INFO(node().get_logger(), "Logging sweep data to %s", csv_logger_.path().c_str());
    }
  }

  transitionTo(ModePhase::SettlingBeforeSweep);
  RCLCPP_INFO(node().get_logger(),
              "Frequency Sweep activated. Holding reference NED [%.2f, %.2f, %.2f], yaw %.2f "
              "rad before excitation.",
              reference_position_ned_m_[0], reference_position_ned_m_[1],
              reference_position_ned_m_[2], reference_yaw_ned_rad_);
}

void FrequencySweepMode::onDeactivate()
{
  csv_logger_.close();
  velocity_integrator_.reset();
  phase_ = ModePhase::Inactive;
  reference_initialized_ = false;
  RCLCPP_INFO(node().get_logger(), "Frequency Sweep deactivated");
}

void FrequencySweepMode::updateSetpoint(float dt_s)
{
  if (phase_ == ModePhase::Inactive) {
    return;
  }

  if (phase_ != ModePhase::FailureHold && !telemetryValid()) {
    enterFailureHold("Local position or attitude telemetry became invalid");
    return;
  }

  if (parameters_.safety.enabled && phase_ != ModePhase::FailureHold &&
      phase_ != ModePhase::CompletedHold) {
    if (const auto violation = safetyViolation()) {
      if (parameters_.safety.abort_on_violation) {
        enterFailureHold(*violation);
        return;
      }

      const rclcpp::Time now = node().get_clock()->now();
      if (last_safety_warning_time_.nanoseconds() == 0 ||
          (now - last_safety_warning_time_).seconds() >= 1.0) {
        RCLCPP_WARN(node().get_logger(), "Safety limit exceeded but abort is disabled: %s",
                    violation->c_str());
        last_safety_warning_time_ = now;
      }
    }
  }

  const float phase_elapsed_s =
      static_cast<float>((node().get_clock()->now() - phase_start_time_).seconds());
  const float max_control_dt_s = 2.F / parameters_.mode.update_rate_hz;
  const float bounded_dt_s =
      std::isfinite(dt_s) ? std::clamp(dt_s, 0.F, max_control_dt_s) : 0.F;

  switch (phase_) {
    case ModePhase::SettlingBeforeSweep:
    case ModePhase::SettlingBetweenSweeps: {
      const TrajectoryCommand command =
          holdCommand(reference_position_ned_m_, reference_yaw_ned_rad_);
      publishAndLog(command, SweepSample{}, phase_elapsed_s, dt_s);

      if (referenceReached()) {
        stable_time_s_ += bounded_dt_s;
        const float required_stable_time_s =
            phase_ == ModePhase::SettlingBeforeSweep ? parameters_.sweep.settle_before_s
                                                     : parameters_.sweep.settle_between_s;
        if (stable_time_s_ >= required_stable_time_s) {
          startSweep();
        }
      } else {
        stable_time_s_ = 0.F;
      }

      if ((phase_ == ModePhase::SettlingBeforeSweep ||
           phase_ == ModePhase::SettlingBetweenSweeps) &&
          phase_elapsed_s > parameters_.sweep.settling_timeout_s) {
        enterFailureHold("Timed out waiting for the reference hover point");
      }
      break;
    }

    case ModePhase::Sweeping: {
      const SweepSample sample =
          sweep_generator_.sample(phase_elapsed_s, currentStage().amplitude);
      const TrajectoryCommand command = sweepCommand(sample, bounded_dt_s, phase_elapsed_s);
      publishAndLog(command, sample, phase_elapsed_s, dt_s);

      if (sample.finished) {
        ++repetition_index_;
        RCLCPP_INFO(node().get_logger(), "Stage '%s' repetition %d/%d complete",
                    currentStage().name.c_str(), repetition_index_,
                    parameters_.sweep.repetitions);
        if (repetition_index_ >= parameters_.sweep.repetitions) {
          if (advanceToNextStageOrFinish()) {
            velocity_integrator_.reset();
            transitionTo(ModePhase::SettlingBetweenSweeps);
          }
        } else {
          velocity_integrator_.reset();
          transitionTo(ModePhase::SettlingBetweenSweeps);
        }
      }
      break;
    }

    case ModePhase::CompletedHold:
      publish(holdCommand(reference_position_ned_m_, reference_yaw_ned_rad_));
      break;

    case ModePhase::FailureHold:
      if (reference_initialized_) {
        publish(holdCommand(failure_hold_position_ned_m_, failure_hold_yaw_ned_rad_));
      }
      break;

    case ModePhase::Inactive:
      break;
  }
}

void FrequencySweepMode::transitionTo(ModePhase phase)
{
  phase_ = phase;
  phase_start_time_ = node().get_clock()->now();
  stable_time_s_ = 0.F;
}

const SweepStage& FrequencySweepMode::currentStage() const
{
  // stage_index_ is only advanced by advanceToNextStageOrFinish(), which stops at the last stage,
  // and declareAndLoadParameters() rejects an empty stage list.
  return parameters_.stages.at(stage_index_);
}

// Returns true when another stage is queued, false when the whole sequence is done (in which case
// the mode has already been moved into CompletedHold).
bool FrequencySweepMode::advanceToNextStageOrFinish()
{
  if (stage_index_ + 1 >= parameters_.stages.size()) {
    finishSuccessfully();
    return false;
  }
  ++stage_index_;
  repetition_index_ = 0;
  RCLCPP_INFO(node().get_logger(), "Advancing to stage %zu/%zu: '%s'", stage_index_ + 1,
              parameters_.stages.size(), currentStage().name.c_str());
  return true;
}

void FrequencySweepMode::startSweep()
{
  velocity_integrator_.reset();
  transitionTo(ModePhase::Sweeping);
  RCLCPP_INFO(node().get_logger(),
              "Starting stage %zu/%zu '%s' repetition %d/%d: %s, %.3f..%.3f Hz, amplitude %.3f",
              stage_index_ + 1, parameters_.stages.size(), currentStage().name.c_str(),
              repetition_index_ + 1, parameters_.sweep.repetitions,
              toString(currentStage().target).c_str(), parameters_.sweep.start_frequency_hz,
              parameters_.sweep.end_frequency_hz, currentStage().amplitude);
}

void FrequencySweepMode::finishSuccessfully()
{
  transitionTo(ModePhase::CompletedHold);
  publish(holdCommand(reference_position_ned_m_, reference_yaw_ned_rad_));
  csv_logger_.close();
  if (!completion_reported_) {
    completed(px4_ros2::Result::Success);
    completion_reported_ = true;
  }
  RCLCPP_INFO(node().get_logger(),
              "All sweep repetitions completed. Holding the reference until another mode is "
              "selected.");
}

void FrequencySweepMode::enterFailureHold(const std::string& reason)
{
  if (telemetryValid()) {
    failure_hold_position_ned_m_ = toArray(local_position_->positionNed());
    failure_hold_yaw_ned_rad_ = attitude_->yaw();
    reference_initialized_ = true;
  }

  transitionTo(ModePhase::FailureHold);
  velocity_integrator_.reset();
  if (reference_initialized_) {
    publish(holdCommand(failure_hold_position_ned_m_, failure_hold_yaw_ned_rad_));
  }
  csv_logger_.close();
  if (!completion_reported_) {
    completed(px4_ros2::Result::ModeFailureOther);
    completion_reported_ = true;
  }
  RCLCPP_ERROR(node().get_logger(), "Frequency Sweep aborted: %s", reason.c_str());
}

bool FrequencySweepMode::telemetryValid() const
{
  if (!local_position_ || !attitude_) {
    return false;
  }
  const auto timeout = telemetryTimeout(parameters_);
  return local_position_->lastValid(timeout) && local_position_->last().xy_valid &&
         local_position_->last().z_valid && local_position_->last().v_xy_valid &&
         local_position_->last().v_z_valid && attitude_->lastValid(timeout);
}

bool FrequencySweepMode::referenceReached() const
{
  const Eigen::Vector3f reference{reference_position_ned_m_[0], reference_position_ned_m_[1],
                                  reference_position_ned_m_[2]};
  return (local_position_->positionNed() - reference).norm() <=
             parameters_.sweep.position_tolerance_m &&
         local_position_->velocityNed().norm() <= parameters_.sweep.velocity_tolerance_m_s;
}

std::optional<std::string> FrequencySweepMode::safetyViolation() const
{
  if (!telemetryValid()) {
    return "telemetry is stale or estimator validity flags are false";
  }

  const Eigen::Vector3f position = local_position_->positionNed();
  const float horizontal_deviation_m =
      std::hypot(position.x() - reference_position_ned_m_[0],
                 position.y() - reference_position_ned_m_[1]);
  if (horizontal_deviation_m > parameters_.safety.max_horizontal_deviation_m) {
    return "horizontal deviation is " + std::to_string(horizontal_deviation_m) + " m";
  }

  const float vertical_deviation_m =
      std::abs(position.z() - reference_position_ned_m_[2]);
  if (vertical_deviation_m > parameters_.safety.max_vertical_deviation_m) {
    return "vertical deviation is " + std::to_string(vertical_deviation_m) + " m";
  }

  const float speed_m_s = local_position_->velocityNed().norm();
  if (speed_m_s > parameters_.safety.max_speed_m_s) {
    return "speed is " + std::to_string(speed_m_s) + " m/s";
  }

  const Eigen::Quaternionf attitude = attitude_->attitude().normalized();
  const float body_z_dot_ned_z =
      std::clamp(1.F - 2.F * (attitude.x() * attitude.x() + attitude.y() * attitude.y()),
                 -1.F, 1.F);
  const float tilt_deg = std::acos(body_z_dot_ned_z) * 180.F / kPi;
  if (tilt_deg > parameters_.safety.max_tilt_deg) {
    return "tilt is " + std::to_string(tilt_deg) + " deg";
  }

  return std::nullopt;
}

TrajectoryCommand FrequencySweepMode::holdCommand(
    const std::array<float, 3>& position_ned_m, float yaw_ned_rad) const
{
  TrajectoryCommand command;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    command.position_ned_m[axis] = position_ned_m[axis];
  }
  command.yaw_ned_rad = wrapPi(yaw_ned_rad);
  return command;
}

TrajectoryCommand FrequencySweepMode::sweepCommand(const SweepSample& sample, float dt_s,
                                                    float sweep_elapsed_s)
{
  const SweepStage& stage = currentStage();

  TrajectoryCommand command;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (stage.position_enabled[axis]) {
      command.position_ned_m[axis] =
          reference_position_ned_m_[axis] + stage.position_offset_ned_m[axis];
    }
    if (stage.velocity_enabled[axis]) {
      command.velocity_ned_m_s[axis] = stage.velocity_base_ned_m_s[axis];
    }
    if (stage.acceleration_enabled[axis]) {
      command.acceleration_ned_m_s2[axis] = stage.acceleration_base_ned_m_s2[axis];
    }
  }
  if (stage.yaw_enabled) {
    command.yaw_ned_rad = wrapPi(reference_yaw_ned_rad_ + stage.yaw_offset_rad);
  }
  if (stage.yaw_rate_enabled) {
    command.yaw_rate_ned_rad_s = stage.yaw_rate_base_rad_s;
  }

  std::array<float, 3> acceleration_excitation{};
  if (isAccelerationTarget(stage.target)) {
    if (isHorizontalTarget(stage.target)) {
      const auto direction = horizontalDirectionNed(
          stage.target, parameters_.sweep.horizontal_frame, reference_yaw_ned_rad_);
      acceleration_excitation[0] = direction[0] * sample.value;
      acceleration_excitation[1] = direction[1] * sample.value;
    } else {
      acceleration_excitation[targetAxis(stage.target)] = sample.value;
    }
  }
  const auto& integrated_velocity =
      velocity_integrator_.update(acceleration_excitation, dt_s, sweep_elapsed_s);
  // The integral is published only where the stage asks for it. An axis can stay velocity-
  // controlled at its baseline without receiving the excitation integral.
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (stage.velocity_enabled[axis] && stage.integrator_publish_enabled[axis]) {
      command.velocity_ned_m_s[axis] =
          stage.velocity_base_ned_m_s[axis] + integrated_velocity[axis];
    }
  }

  // The excited component is written last so it overrides the baseline set above. In the heading
  // frame a horizontal excitation is spread over both NED components, so both get rewritten.
  const bool yaw_target = isYawTarget(stage.target);
  const std::size_t target_axis = yaw_target ? 0 : targetAxis(stage.target);
  const bool spread_horizontally = !yaw_target && target_axis < 2 &&
                                   parameters_.sweep.horizontal_frame == HorizontalFrame::Heading;
  const std::array<float, 2> direction =
      spread_horizontally ? horizontalDirectionNed(stage.target,
                                                   parameters_.sweep.horizontal_frame,
                                                   reference_yaw_ned_rad_)
                          : std::array<float, 2>{0.F, 0.F};

  const auto applyExcitation = [&](std::array<std::optional<float>, 3>& values, auto&& baseline) {
    if (spread_horizontally) {
      for (std::size_t axis = 0; axis < 2; ++axis) {
        setComponent(values, axis, baseline(axis) + direction[axis] * sample.value);
      }
    } else {
      setComponent(values, target_axis, baseline(target_axis) + sample.value);
    }
  };

  switch (stage.target) {
    case ExcitationTarget::PositionX:
    case ExcitationTarget::PositionY:
    case ExcitationTarget::PositionZ:
      applyExcitation(command.position_ned_m, [&](std::size_t axis) {
        return reference_position_ned_m_[axis] + stage.position_offset_ned_m[axis];
      });
      break;
    case ExcitationTarget::VelocityX:
    case ExcitationTarget::VelocityY:
    case ExcitationTarget::VelocityZ:
      applyExcitation(command.velocity_ned_m_s, [&](std::size_t axis) {
        return stage.velocity_base_ned_m_s[axis] +
               (stage.integrator_publish_enabled[axis] ? integrated_velocity[axis] : 0.F);
      });
      break;
    case ExcitationTarget::AccelerationX:
    case ExcitationTarget::AccelerationY:
    case ExcitationTarget::AccelerationZ:
      applyExcitation(command.acceleration_ned_m_s2, [&](std::size_t axis) {
        return stage.acceleration_base_ned_m_s2[axis];
      });
      break;
    case ExcitationTarget::Yaw:
      command.yaw_ned_rad =
          wrapPi(reference_yaw_ned_rad_ + stage.yaw_offset_rad + sample.value);
      break;
    case ExcitationTarget::YawRate:
      command.yaw_rate_ned_rad_s = stage.yaw_rate_base_rad_s + sample.value;
      break;
  }

  return command;
}

void FrequencySweepMode::publish(const TrajectoryCommand& command)
{
  px4_ros2::TrajectorySetpoint setpoint;
  if (command.position_ned_m[0]) setpoint.withPositionX(*command.position_ned_m[0]);
  if (command.position_ned_m[1]) setpoint.withPositionY(*command.position_ned_m[1]);
  if (command.position_ned_m[2]) setpoint.withPositionZ(*command.position_ned_m[2]);
  if (command.velocity_ned_m_s[0]) setpoint.withVelocityX(*command.velocity_ned_m_s[0]);
  if (command.velocity_ned_m_s[1]) setpoint.withVelocityY(*command.velocity_ned_m_s[1]);
  if (command.velocity_ned_m_s[2]) setpoint.withVelocityZ(*command.velocity_ned_m_s[2]);
  if (command.acceleration_ned_m_s2[0]) {
    setpoint.withAccelerationX(*command.acceleration_ned_m_s2[0]);
  }
  if (command.acceleration_ned_m_s2[1]) {
    setpoint.withAccelerationY(*command.acceleration_ned_m_s2[1]);
  }
  if (command.acceleration_ned_m_s2[2]) {
    setpoint.withAccelerationZ(*command.acceleration_ned_m_s2[2]);
  }
  if (command.yaw_ned_rad) setpoint.withYaw(*command.yaw_ned_rad);
  if (command.yaw_rate_ned_rad_s) setpoint.withYawRate(*command.yaw_rate_ned_rad_s);
  trajectory_setpoint_->update(setpoint);
}

void FrequencySweepMode::publishAndLog(const TrajectoryCommand& command,
                                       const SweepSample& sample, float phase_elapsed_s,
                                       float dt_s)
{
  publish(command);
  if (csv_logger_.isOpen() && telemetryValid()) {
    csv_logger_.write(node().get_clock()->now().seconds(), dt_s, phase_, currentStage(),
                      repetition_index_, phase_elapsed_s, sample, command, telemetrySnapshot());
  }
}

TelemetrySnapshot FrequencySweepMode::telemetrySnapshot() const
{
  TelemetrySnapshot snapshot;
  snapshot.position_ned_m = toArray(local_position_->positionNed());
  snapshot.velocity_ned_m_s = toArray(local_position_->velocityNed());
  snapshot.attitude_rpy_rad = {attitude_->roll(), attitude_->pitch(), attitude_->yaw()};

  const float nan = std::numeric_limits<float>::quiet_NaN();
  snapshot.angular_velocity_frd_rad_s = {nan, nan, nan};
  if (angular_velocity_->lastValid(telemetryTimeout(parameters_))) {
    snapshot.angular_velocity_frd_rad_s = toArray(angular_velocity_->angularVelocityFrd());
    // The accessors above drop the message header. Keep the PX4-clock timestamps so each row can
    // be located in the ulog, and so the sample-to-receive gap gives the offboard link delay.
    const auto& message = angular_velocity_->last();
    snapshot.px4_timestamp_us = message.timestamp;
    snapshot.px4_timestamp_sample_us = message.timestamp_sample;
  }
  return snapshot;
}

void FrequencySweepMode::setComponent(std::array<std::optional<float>, 3>& values,
                                      std::size_t axis, float value)
{
  values.at(axis) = value;
}

float FrequencySweepMode::wrapPi(float angle_rad)
{
  return std::remainder(angle_rad, 2.F * kPi);
}

}  // namespace px4_frequency_sweep
