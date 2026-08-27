#include "px4_frequency_sweep/configuration.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

#ifndef PX4_FREQUENCY_SWEEP_PACKAGE_DIR
// Only when built outside the package CMake target; relative log paths then use the cwd.
#define PX4_FREQUENCY_SWEEP_PACKAGE_DIR ""
#endif

namespace px4_frequency_sweep {
namespace {

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return value;
}

float declareFloat(rclcpp::Node& node, const std::string& name, float default_value)
{
  return static_cast<float>(node.declare_parameter<double>(name, default_value));
}


std::string normaliseTopicNamespacePrefix(std::string prefix)
{
  while (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  if (!prefix.empty() && prefix.front() != '/') {
    prefix.insert(prefix.begin(), '/');
  }
  return prefix;
}

std::array<float, 3> declareFloatArray3(rclcpp::Node& node, const std::string& name,
                                       const std::array<float, 3>& default_value)
{
  const std::vector<double> defaults{default_value[0], default_value[1], default_value[2]};
  const auto values = node.declare_parameter<std::vector<double>>(name, defaults);
  if (values.size() != 3) {
    throw std::invalid_argument("Parameter '" + name + "' must contain exactly three values");
  }
  return {static_cast<float>(values[0]), static_cast<float>(values[1]),
          static_cast<float>(values[2])};
}

std::array<bool, 3> declareBoolArray3(rclcpp::Node& node, const std::string& name,
                                     const std::array<bool, 3>& default_value)
{
  const std::vector<bool> defaults{default_value[0], default_value[1], default_value[2]};
  const auto values = node.declare_parameter<std::vector<bool>>(name, defaults);
  if (values.size() != 3) {
    throw std::invalid_argument("Parameter '" + name + "' must contain exactly three values");
  }
  return {values[0], values[1], values[2]};
}

void requirePositive(const std::string& name, float value)
{
  if (!std::isfinite(value) || value <= 0.F) {
    throw std::invalid_argument("Parameter '" + name + "' must be finite and greater than zero");
  }
}

void requireNonNegative(const std::string& name, float value)
{
  if (!std::isfinite(value) || value < 0.F) {
    throw std::invalid_argument("Parameter '" + name + "' must be finite and non-negative");
  }
}

void requireFinite(const std::string& name, float value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("Parameter '" + name + "' must be finite");
  }
}

void requireFinite(const std::string& name, const std::array<float, 3>& values)
{
  for (float value : values) {
    requireFinite(name, value);
  }
}

// PX4's PositionControl::_inputValid() rejects the whole setpoint unless X and Y of each field
// are both finite or both NaN. Checking here turns a mid-flight failsafe into a startup error.
void requireHorizontalPairing(const std::string& stage_name, const std::string& field,
                              const std::array<bool, 3>& enabled)
{
  if (enabled[0] != enabled[1]) {
    throw std::invalid_argument(
        "stages." + stage_name + "." + field +
        " must enable X and Y together: PX4 requires the horizontal components of each "
        "TrajectorySetpoint field to be valid or NaN in pairs, and rejects the entire setpoint "
        "otherwise");
  }
}

// Stage names come from sweep.sequence, so stages.<name>.* cannot be declared up front.
std::vector<SweepStage> declareStages(rclcpp::Node& node)
{
  const auto sequence =
      node.declare_parameter<std::vector<std::string>>("sweep.sequence", std::vector<std::string>{});
  if (sequence.empty()) {
    throw std::invalid_argument(
        "sweep.sequence must list at least one stage name, e.g. [\"roll\", \"pitch\", \"yaw\"]");
  }

  std::vector<SweepStage> stages;
  stages.reserve(sequence.size());
  for (const std::string& name : sequence) {
    if (name.empty()) {
      throw std::invalid_argument("sweep.sequence contains an empty stage name");
    }
    const std::string prefix = "stages." + name + ".";
    SweepStage stage;
    stage.name = name;

    stage.target = excitationTargetFromString(
        node.declare_parameter<std::string>(prefix + "target", toString(stage.target)));
    stage.amplitude = declareFloat(node, prefix + "amplitude", stage.amplitude);

    stage.position_enabled =
        declareBoolArray3(node, prefix + "position_enabled", stage.position_enabled);
    stage.velocity_enabled =
        declareBoolArray3(node, prefix + "velocity_enabled", stage.velocity_enabled);
    stage.acceleration_enabled =
        declareBoolArray3(node, prefix + "acceleration_enabled", stage.acceleration_enabled);
    stage.integrator_publish_enabled = declareBoolArray3(
        node, prefix + "integrator_publish_enabled", stage.integrator_publish_enabled);
    stage.yaw_enabled = node.declare_parameter<bool>(prefix + "yaw_enabled", stage.yaw_enabled);
    stage.yaw_rate_enabled =
        node.declare_parameter<bool>(prefix + "yaw_rate_enabled", stage.yaw_rate_enabled);

    stage.position_offset_ned_m =
        declareFloatArray3(node, prefix + "position_offset_ned_m", stage.position_offset_ned_m);
    stage.velocity_base_ned_m_s =
        declareFloatArray3(node, prefix + "velocity_base_ned_m_s", stage.velocity_base_ned_m_s);
    stage.acceleration_base_ned_m_s2 = declareFloatArray3(
        node, prefix + "acceleration_base_ned_m_s2", stage.acceleration_base_ned_m_s2);
    stage.yaw_offset_rad = declareFloat(node, prefix + "yaw_offset_rad", stage.yaw_offset_rad);
    stage.yaw_rate_base_rad_s =
        declareFloat(node, prefix + "yaw_rate_base_rad_s", stage.yaw_rate_base_rad_s);

    stages.push_back(std::move(stage));
  }
  return stages;
}

void validateStage(const SweepStage& stage)
{
  const std::string& name = stage.name;
  requireFinite("stages." + name + ".amplitude", stage.amplitude);

  requireHorizontalPairing(name, "position_enabled", stage.position_enabled);
  requireHorizontalPairing(name, "velocity_enabled", stage.velocity_enabled);
  requireHorizontalPairing(name, "acceleration_enabled", stage.acceleration_enabled);

  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!stage.position_enabled[axis] && !stage.velocity_enabled[axis] &&
        !stage.acceleration_enabled[axis]) {
      throw std::invalid_argument("stages." + name + " leaves axis " + std::to_string(axis) +
                                  " with no setpoint; PX4 requires at least one of position, "
                                  "velocity or acceleration per axis");
    }
    if (stage.integrator_publish_enabled[axis] && !stage.velocity_enabled[axis]) {
      throw std::invalid_argument("stages." + name + ".integrator_publish_enabled[" +
                                  std::to_string(axis) +
                                  "] requires velocity_enabled on the same axis");
    }
  }

  const bool target_published =
      isYawTarget(stage.target)
          ? (stage.target == ExcitationTarget::Yaw ? stage.yaw_enabled : stage.yaw_rate_enabled)
          : (isPositionTarget(stage.target) ? stage.position_enabled[targetAxis(stage.target)]
             : isVelocityTarget(stage.target)
                 ? stage.velocity_enabled[targetAxis(stage.target)]
                 : stage.acceleration_enabled[targetAxis(stage.target)]);
  if (!target_published) {
    throw std::invalid_argument("stages." + name + " excites '" + toString(stage.target) +
                                "' but the matching setpoint component is disabled, so the "
                                "excitation would never be published");
  }

  requireFinite("stages." + name + ".position_offset_ned_m", stage.position_offset_ned_m);
  requireFinite("stages." + name + ".velocity_base_ned_m_s", stage.velocity_base_ned_m_s);
  requireFinite("stages." + name + ".acceleration_base_ned_m_s2",
                stage.acceleration_base_ned_m_s2);
  requireFinite("stages." + name + ".yaw_offset_rad", stage.yaw_offset_rad);
  requireFinite("stages." + name + ".yaw_rate_base_rad_s", stage.yaw_rate_base_rad_s);
}

void validate(const FrequencySweepParameters& parameters)
{
  if (parameters.mode.name.empty() || parameters.mode.name.size() >= 25) {
    throw std::invalid_argument("mode.name must contain between 1 and 24 characters");
  }
  requirePositive("mode.update_rate_hz", parameters.mode.update_rate_hz);

  requireFinite("reference.configured_position_ned_m",
                parameters.reference.configured_position_ned_m);
  requireFinite("reference.configured_yaw_ned_rad", parameters.reference.configured_yaw_ned_rad);
  requirePositive("reference.max_transit_distance_m",
                  parameters.reference.max_transit_distance_m);
  requirePositive("reference.transit_timeout_s", parameters.reference.transit_timeout_s);

  if (parameters.stages.empty()) {
    throw std::invalid_argument("sweep.sequence must name at least one stage");
  }
  for (const SweepStage& stage : parameters.stages) {
    validateStage(stage);
  }

  requirePositive("sweep.start_frequency_hz", parameters.sweep.start_frequency_hz);
  requirePositive("sweep.end_frequency_hz", parameters.sweep.end_frequency_hz);
  if (parameters.sweep.end_frequency_hz < parameters.sweep.start_frequency_hz) {
    throw std::invalid_argument("sweep.end_frequency_hz must be >= sweep.start_frequency_hz");
  }
  requirePositive("sweep.duration_s", parameters.sweep.duration_s);
  requireFinite("sweep.phase_offset_rad", parameters.sweep.phase_offset_rad);
  requireNonNegative("sweep.fade_in_s", parameters.sweep.fade_in_s);
  requireNonNegative("sweep.fade_out_s", parameters.sweep.fade_out_s);
  if (parameters.sweep.fade_in_s + parameters.sweep.fade_out_s > parameters.sweep.duration_s) {
    throw std::invalid_argument("sweep.fade_in_s + sweep.fade_out_s must not exceed duration");
  }
  if (parameters.sweep.repetitions < 1) {
    throw std::invalid_argument("sweep.repetitions must be at least one");
  }
  requireNonNegative("sweep.settle_before_s", parameters.sweep.settle_before_s);
  requireNonNegative("sweep.settle_between_s", parameters.sweep.settle_between_s);
  requirePositive("sweep.settling_timeout_s", parameters.sweep.settling_timeout_s);
  requirePositive("sweep.position_tolerance_m", parameters.sweep.position_tolerance_m);
  requirePositive("sweep.velocity_tolerance_m_s", parameters.sweep.velocity_tolerance_m_s);
  requirePositive("sweep.minimum_samples_per_cycle",
                  parameters.sweep.minimum_samples_per_cycle);
  if (parameters.mode.update_rate_hz <
      parameters.sweep.end_frequency_hz * parameters.sweep.minimum_samples_per_cycle) {
    throw std::invalid_argument(
        "mode.update_rate_hz is too low for sweep.end_frequency_hz and "
        "sweep.minimum_samples_per_cycle");
  }

  requireNonNegative("velocity_integrator.start_delay_s",
                     parameters.velocity_integrator.start_delay_s);
  requirePositive("velocity_integrator.leak_time_constant_s",
                  parameters.velocity_integrator.leak_time_constant_s);
  requirePositive("velocity_integrator.max_abs_velocity_m_s",
                  parameters.velocity_integrator.max_abs_velocity_m_s);

  requirePositive("safety.max_horizontal_deviation_m",
                  parameters.safety.max_horizontal_deviation_m);
  requirePositive("safety.max_vertical_deviation_m",
                  parameters.safety.max_vertical_deviation_m);
  requirePositive("safety.max_speed_m_s", parameters.safety.max_speed_m_s);
  requirePositive("safety.max_tilt_deg", parameters.safety.max_tilt_deg);
  if (parameters.safety.max_tilt_deg >= 90.F) {
    throw std::invalid_argument("safety.max_tilt_deg must be less than 90 degrees");
  }
  requirePositive("safety.telemetry_timeout_s", parameters.safety.telemetry_timeout_s);

  if (parameters.logging.directory.empty()) {
    throw std::invalid_argument("logging.directory must not be empty");
  }
  if (!std::filesystem::path(parameters.logging.directory).is_absolute()) {
    throw std::logic_error("logging.directory should have been made absolute before validation");
  }
  if (parameters.logging.flush_every_n_samples < 1) {
    throw std::invalid_argument("logging.flush_every_n_samples must be at least one");
  }
}

// A relative directory resolves against the package source tree, not the launch cwd, so 'logs'
// always lands in Px4FrequencySweep/logs. Absolute values are honoured as given.
std::string resolveLogDirectory(const std::string& configured)
{
  const std::filesystem::path path(configured);
  if (path.is_absolute()) {
    return configured;
  }

  const std::filesystem::path package_dir(PX4_FREQUENCY_SWEEP_PACKAGE_DIR);
  if (package_dir.empty()) {
    return std::filesystem::absolute(path).lexically_normal().string();
  }
  return (package_dir / path).lexically_normal().string();
}

}  // namespace

px4_ros2::ModeBase::Settings declareModeSettings(rclcpp::Node& node)
{
  const auto name = node.declare_parameter<std::string>("mode.name", "Frequency Sweep");
  const bool prevent_arming = node.declare_parameter<bool>("mode.prevent_arming", true);
  const bool activate_disarmed =
      node.declare_parameter<bool>("mode.activate_even_while_disarmed", false);

  px4_ros2::ModeBase::Settings settings{name};
  settings.preventArming(prevent_arming).activateEvenWhileDisarmed(activate_disarmed);
  return settings;
}

std::string declareTopicNamespacePrefix(rclcpp::Node& node)
{
  return normaliseTopicNamespacePrefix(
      node.declare_parameter<std::string>("px4_topic_namespace_prefix", ""));
}

FrequencySweepParameters declareAndLoadParameters(rclcpp::Node& node)
{
  FrequencySweepParameters parameters;

  node.get_parameter("mode.name", parameters.mode.name);
  node.get_parameter("mode.prevent_arming", parameters.mode.prevent_arming);
  node.get_parameter("mode.activate_even_while_disarmed",
                     parameters.mode.activate_even_while_disarmed);
  // Already declared by declareTopicNamespacePrefix() via the ModeBase constructor argument.
  std::string topic_namespace_prefix;
  node.get_parameter("px4_topic_namespace_prefix", topic_namespace_prefix);
  parameters.mode.topic_namespace_prefix =
      normaliseTopicNamespacePrefix(std::move(topic_namespace_prefix));
  parameters.mode.update_rate_hz =
      declareFloat(node, "mode.update_rate_hz", parameters.mode.update_rate_hz);

  parameters.reference.position_source = referenceSourceFromString(node.declare_parameter<std::string>(
      "reference.position_source", toString(parameters.reference.position_source)));
  parameters.reference.configured_position_ned_m = declareFloatArray3(
      node, "reference.configured_position_ned_m",
      parameters.reference.configured_position_ned_m);
  parameters.reference.yaw_source = referenceSourceFromString(node.declare_parameter<std::string>(
      "reference.yaw_source", toString(parameters.reference.yaw_source)));
  parameters.reference.configured_yaw_ned_rad = declareFloat(
      node, "reference.configured_yaw_ned_rad", parameters.reference.configured_yaw_ned_rad);
  parameters.reference.max_transit_distance_m = declareFloat(
      node, "reference.max_transit_distance_m", parameters.reference.max_transit_distance_m);
  parameters.reference.transit_timeout_s = declareFloat(
      node, "reference.transit_timeout_s", parameters.reference.transit_timeout_s);
  // rclcpp ignores unknown keys, so reject the renamed max_initial_offset_m explicitly rather
  // than let a stale config silently lose its limit.
  if (node.get_node_options().parameter_overrides().end() !=
      std::find_if(node.get_node_options().parameter_overrides().begin(),
                   node.get_node_options().parameter_overrides().end(),
                   [](const rclcpp::Parameter& parameter) {
                     return parameter.get_name() == "reference.max_initial_offset_m";
                   })) {
    throw std::invalid_argument(
        "reference.max_initial_offset_m was replaced by reference.max_transit_distance_m. It used "
        "to reject a reference the aircraft was not already near; the mode now flies to the "
        "reference, so the limit is a transit budget. Rename the key and raise the value.");
  }

  parameters.stages = declareStages(node);

  parameters.sweep.horizontal_frame = horizontalFrameFromString(
      node.declare_parameter<std::string>("sweep.horizontal_frame",
                                          toString(parameters.sweep.horizontal_frame)));
  parameters.sweep.waveform = waveformFromString(node.declare_parameter<std::string>(
      "sweep.waveform", toString(parameters.sweep.waveform)));
  parameters.sweep.start_frequency_hz = declareFloat(
      node, "sweep.start_frequency_hz", parameters.sweep.start_frequency_hz);
  parameters.sweep.end_frequency_hz =
      declareFloat(node, "sweep.end_frequency_hz", parameters.sweep.end_frequency_hz);
  parameters.sweep.duration_s =
      declareFloat(node, "sweep.duration_s", parameters.sweep.duration_s);
  parameters.sweep.phase_offset_rad =
      declareFloat(node, "sweep.phase_offset_rad", parameters.sweep.phase_offset_rad);
  parameters.sweep.fade_in_s =
      declareFloat(node, "sweep.fade_in_s", parameters.sweep.fade_in_s);
  parameters.sweep.fade_out_s =
      declareFloat(node, "sweep.fade_out_s", parameters.sweep.fade_out_s);
  parameters.sweep.repetitions = static_cast<int>(node.declare_parameter<int64_t>(
      "sweep.repetitions", parameters.sweep.repetitions));
  parameters.sweep.settle_before_s =
      declareFloat(node, "sweep.settle_before_s", parameters.sweep.settle_before_s);
  parameters.sweep.settle_between_s =
      declareFloat(node, "sweep.settle_between_s", parameters.sweep.settle_between_s);
  parameters.sweep.settling_timeout_s =
      declareFloat(node, "sweep.settling_timeout_s", parameters.sweep.settling_timeout_s);
  parameters.sweep.position_tolerance_m =
      declareFloat(node, "sweep.position_tolerance_m", parameters.sweep.position_tolerance_m);
  parameters.sweep.velocity_tolerance_m_s = declareFloat(
      node, "sweep.velocity_tolerance_m_s", parameters.sweep.velocity_tolerance_m_s);
  parameters.sweep.minimum_samples_per_cycle = declareFloat(
      node, "sweep.minimum_samples_per_cycle",
      parameters.sweep.minimum_samples_per_cycle);

  parameters.velocity_integrator.enabled = node.declare_parameter<bool>(
      "velocity_integrator.enabled", parameters.velocity_integrator.enabled);
  parameters.velocity_integrator.start_delay_s = declareFloat(
      node, "velocity_integrator.start_delay_s",
      parameters.velocity_integrator.start_delay_s);
  parameters.velocity_integrator.leak_time_constant_s = declareFloat(
      node, "velocity_integrator.leak_time_constant_s",
      parameters.velocity_integrator.leak_time_constant_s);
  parameters.velocity_integrator.max_abs_velocity_m_s = declareFloat(
      node, "velocity_integrator.max_abs_velocity_m_s",
      parameters.velocity_integrator.max_abs_velocity_m_s);

  parameters.safety.enabled =
      node.declare_parameter<bool>("safety.enabled", parameters.safety.enabled);
  parameters.safety.abort_on_violation = node.declare_parameter<bool>(
      "safety.abort_on_violation", parameters.safety.abort_on_violation);
  parameters.safety.max_horizontal_deviation_m = declareFloat(
      node, "safety.max_horizontal_deviation_m",
      parameters.safety.max_horizontal_deviation_m);
  parameters.safety.max_vertical_deviation_m = declareFloat(
      node, "safety.max_vertical_deviation_m", parameters.safety.max_vertical_deviation_m);
  parameters.safety.max_speed_m_s =
      declareFloat(node, "safety.max_speed_m_s", parameters.safety.max_speed_m_s);
  parameters.safety.max_tilt_deg =
      declareFloat(node, "safety.max_tilt_deg", parameters.safety.max_tilt_deg);
  parameters.safety.telemetry_timeout_s = declareFloat(
      node, "safety.telemetry_timeout_s", parameters.safety.telemetry_timeout_s);

  parameters.logging.enabled =
      node.declare_parameter<bool>("logging.enabled", parameters.logging.enabled);
  parameters.logging.required =
      node.declare_parameter<bool>("logging.required", parameters.logging.required);
  parameters.logging.directory = node.declare_parameter<std::string>(
      "logging.directory", parameters.logging.directory);
  parameters.logging.flush_every_n_samples = static_cast<int>(node.declare_parameter<int64_t>(
      "logging.flush_every_n_samples", parameters.logging.flush_every_n_samples));
  parameters.logging.directory = resolveLogDirectory(parameters.logging.directory);

  validate(parameters);
  return parameters;
}

ExcitationTarget excitationTargetFromString(const std::string& value)
{
  const std::string normalized = lower(value);
  if (normalized == "position_x") return ExcitationTarget::PositionX;
  if (normalized == "position_y") return ExcitationTarget::PositionY;
  if (normalized == "position_z") return ExcitationTarget::PositionZ;
  if (normalized == "velocity_x") return ExcitationTarget::VelocityX;
  if (normalized == "velocity_y") return ExcitationTarget::VelocityY;
  if (normalized == "velocity_z") return ExcitationTarget::VelocityZ;
  if (normalized == "acceleration_x") return ExcitationTarget::AccelerationX;
  if (normalized == "acceleration_y") return ExcitationTarget::AccelerationY;
  if (normalized == "acceleration_z") return ExcitationTarget::AccelerationZ;
  if (normalized == "yaw") return ExcitationTarget::Yaw;
  if (normalized == "yaw_rate") return ExcitationTarget::YawRate;
  throw std::invalid_argument(
      "sweep.target must be position_x/y/z, velocity_x/y/z, acceleration_x/y/z, yaw, or "
      "yaw_rate");
}

std::string toString(ExcitationTarget target)
{
  switch (target) {
    case ExcitationTarget::PositionX:
      return "position_x";
    case ExcitationTarget::PositionY:
      return "position_y";
    case ExcitationTarget::PositionZ:
      return "position_z";
    case ExcitationTarget::VelocityX:
      return "velocity_x";
    case ExcitationTarget::VelocityY:
      return "velocity_y";
    case ExcitationTarget::VelocityZ:
      return "velocity_z";
    case ExcitationTarget::AccelerationX:
      return "acceleration_x";
    case ExcitationTarget::AccelerationY:
      return "acceleration_y";
    case ExcitationTarget::AccelerationZ:
      return "acceleration_z";
    case ExcitationTarget::Yaw:
      return "yaw";
    case ExcitationTarget::YawRate:
      return "yaw_rate";
  }
  throw std::logic_error("Unhandled excitation target");
}

Waveform waveformFromString(const std::string& value)
{
  const std::string normalized = lower(value);
  if (normalized == "linear") return Waveform::Linear;
  if (normalized == "logarithmic" || normalized == "log") return Waveform::Logarithmic;
  throw std::invalid_argument("sweep.waveform must be 'linear' or 'logarithmic'");
}

std::string toString(Waveform waveform)
{
  return waveform == Waveform::Linear ? "linear" : "logarithmic";
}

ReferenceSource referenceSourceFromString(const std::string& value)
{
  const std::string normalized = lower(value);
  if (normalized == "activation") return ReferenceSource::Activation;
  if (normalized == "configured") return ReferenceSource::Configured;
  throw std::invalid_argument("Reference source must be 'activation' or 'configured'");
}

std::string toString(ReferenceSource source)
{
  return source == ReferenceSource::Activation ? "activation" : "configured";
}

HorizontalFrame horizontalFrameFromString(const std::string& value)
{
  const std::string normalized = lower(value);
  if (normalized == "ned") return HorizontalFrame::Ned;
  if (normalized == "heading" || normalized == "body_heading") return HorizontalFrame::Heading;
  throw std::invalid_argument("sweep.horizontal_frame must be 'ned' or 'heading'");
}

std::string toString(HorizontalFrame frame)
{
  return frame == HorizontalFrame::Ned ? "ned" : "heading";
}

bool isAccelerationTarget(ExcitationTarget target)
{
  return target == ExcitationTarget::AccelerationX ||
         target == ExcitationTarget::AccelerationY ||
         target == ExcitationTarget::AccelerationZ;
}

bool isPositionTarget(ExcitationTarget target)
{
  return target == ExcitationTarget::PositionX || target == ExcitationTarget::PositionY ||
         target == ExcitationTarget::PositionZ;
}

bool isVelocityTarget(ExcitationTarget target)
{
  return target == ExcitationTarget::VelocityX || target == ExcitationTarget::VelocityY ||
         target == ExcitationTarget::VelocityZ;
}

bool isYawTarget(ExcitationTarget target)
{
  return target == ExcitationTarget::Yaw || target == ExcitationTarget::YawRate;
}

std::size_t targetAxis(ExcitationTarget target)
{
  switch (target) {
    case ExcitationTarget::PositionX:
    case ExcitationTarget::VelocityX:
    case ExcitationTarget::AccelerationX:
      return 0;
    case ExcitationTarget::PositionY:
    case ExcitationTarget::VelocityY:
    case ExcitationTarget::AccelerationY:
      return 1;
    case ExcitationTarget::PositionZ:
    case ExcitationTarget::VelocityZ:
    case ExcitationTarget::AccelerationZ:
      return 2;
    case ExcitationTarget::Yaw:
    case ExcitationTarget::YawRate:
      break;
  }
  throw std::invalid_argument("Yaw targets do not have a translational axis");
}

int sysidAxis(ExcitationTarget target)
{
  switch (target) {
    // +Y acceleration is served by rolling right.
    case ExcitationTarget::AccelerationY:
      return 0;
    // +X acceleration is served by pitching forward.
    case ExcitationTarget::AccelerationX:
      return 1;
    case ExcitationTarget::Yaw:
    case ExcitationTarget::YawRate:
      return 2;
    // Vertical excitation lands on thrust, not a rotational axis.
    case ExcitationTarget::PositionZ:
    case ExcitationTarget::VelocityZ:
    case ExcitationTarget::AccelerationZ:
      return 3;
    // Drives the whole cascade; the attitude response maps to no single axis.
    case ExcitationTarget::PositionX:
    case ExcitationTarget::PositionY:
    case ExcitationTarget::VelocityX:
    case ExcitationTarget::VelocityY:
      return -1;
  }
  throw std::logic_error("Unhandled excitation target");
}

}  // namespace px4_frequency_sweep
