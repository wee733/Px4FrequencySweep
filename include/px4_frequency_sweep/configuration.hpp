#pragma once

#include <cstddef>
#include <string>

#include <px4_ros2/components/mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_frequency_sweep/parameters.hpp"

namespace px4_frequency_sweep {

px4_ros2::ModeBase::Settings declareModeSettings(rclcpp::Node& node);

bool isPositionTarget(ExcitationTarget target);
bool isVelocityTarget(ExcitationTarget target);
bool isYawTarget(ExcitationTarget target);

// Normalised to an absolute prefix without a trailing slash; empty means stock '/fmu/...'.
std::string declareTopicNamespacePrefix(rclcpp::Node& node);

FrequencySweepParameters declareAndLoadParameters(rclcpp::Node& node);

ExcitationTarget excitationTargetFromString(const std::string& value);
std::string toString(ExcitationTarget target);
Waveform waveformFromString(const std::string& value);
std::string toString(Waveform waveform);
ReferenceSource referenceSourceFromString(const std::string& value);
std::string toString(ReferenceSource source);
HorizontalFrame horizontalFrameFromString(const std::string& value);
std::string toString(HorizontalFrame frame);

bool isAccelerationTarget(ExcitationTarget target);
std::size_t targetAxis(ExcitationTarget target);

// Rotational axis for the identification tooling: 0=roll, 1=pitch, 2=yaw, 3=thrust, -1 if the
// target maps onto no single rotational axis. Not targetAxis(), which is the NED axis.
int sysidAxis(ExcitationTarget target);

}  // namespace px4_frequency_sweep
