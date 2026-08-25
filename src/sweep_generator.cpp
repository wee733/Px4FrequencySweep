#include "px4_frequency_sweep/sweep_generator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace px4_frequency_sweep {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float halfCosineRamp(float normalized_time)
{
  const float clamped = std::clamp(normalized_time, 0.F, 1.F);
  return 0.5F - 0.5F * std::cos(kPi * clamped);
}

}  // namespace

SweepGenerator::SweepGenerator(SweepParameters parameters) : parameters_(std::move(parameters)) {}

SweepSample SweepGenerator::sample(float elapsed_s) const
{
  const float clamped_time = std::clamp(elapsed_s, 0.F, parameters_.duration_s);
  const float current_envelope = envelope(clamped_time);
  const float phase_rad = phaseRad(clamped_time);

  SweepSample result;
  result.instantaneous_frequency_hz = instantaneousFrequencyHz(clamped_time);
  result.envelope = current_envelope;
  result.value = parameters_.amplitude * current_envelope * std::sin(phase_rad);
  result.finished = elapsed_s >= parameters_.duration_s;
  return result;
}

float SweepGenerator::phaseRad(float elapsed_s) const
{
  if (parameters_.waveform == Waveform::Linear) {
    const float frequency_slope_hz_s =
        (parameters_.end_frequency_hz - parameters_.start_frequency_hz) /
        parameters_.duration_s;
    return 2.F * kPi *
               (parameters_.start_frequency_hz * elapsed_s +
                0.5F * frequency_slope_hz_s * elapsed_s * elapsed_s) +
           parameters_.phase_offset_rad;
  }

  if (std::abs(parameters_.end_frequency_hz - parameters_.start_frequency_hz) < 1e-6F) {
    return 2.F * kPi * parameters_.start_frequency_hz * elapsed_s +
           parameters_.phase_offset_rad;
  }

  const float logarithmic_rate =
      std::log(parameters_.end_frequency_hz / parameters_.start_frequency_hz) /
      parameters_.duration_s;
  return 2.F * kPi * parameters_.start_frequency_hz *
             (std::exp(logarithmic_rate * elapsed_s) - 1.F) / logarithmic_rate +
         parameters_.phase_offset_rad;
}

float SweepGenerator::instantaneousFrequencyHz(float elapsed_s) const
{
  const float progress = elapsed_s / parameters_.duration_s;
  if (parameters_.waveform == Waveform::Linear) {
    return parameters_.start_frequency_hz +
           (parameters_.end_frequency_hz - parameters_.start_frequency_hz) * progress;
  }
  return parameters_.start_frequency_hz *
         std::pow(parameters_.end_frequency_hz / parameters_.start_frequency_hz, progress);
}

float SweepGenerator::envelope(float elapsed_s) const
{
  float result = 1.F;
  if (parameters_.fade_in_s > 0.F && elapsed_s < parameters_.fade_in_s) {
    result *= halfCosineRamp(elapsed_s / parameters_.fade_in_s);
  }
  const float remaining_s = parameters_.duration_s - elapsed_s;
  if (parameters_.fade_out_s > 0.F && remaining_s < parameters_.fade_out_s) {
    result *= halfCosineRamp(remaining_s / parameters_.fade_out_s);
  }
  return result;
}

LeakyVelocityIntegrator::LeakyVelocityIntegrator(VelocityIntegratorParameters parameters)
    : parameters_(std::move(parameters))
{
  reset();
}

void LeakyVelocityIntegrator::reset()
{
  velocity_ned_m_s_.fill(0.F);
  previous_acceleration_ned_m_s2_.fill(0.F);
  has_previous_sample_ = false;
}

const std::array<float, 3>& LeakyVelocityIntegrator::update(
    const std::array<float, 3>& acceleration_excitation_ned_m_s2, float dt_s,
    float sweep_elapsed_s)
{
  if (!parameters_.enabled || dt_s <= 0.F || !std::isfinite(dt_s)) {
    return velocity_ned_m_s_;
  }

  if (sweep_elapsed_s < parameters_.start_delay_s) {
    previous_acceleration_ned_m_s2_ = acceleration_excitation_ned_m_s2;
    has_previous_sample_ = true;
    return velocity_ned_m_s_;
  }

  const float leak = std::exp(-dt_s / parameters_.leak_time_constant_s);
  for (std::size_t axis = 0; axis < velocity_ned_m_s_.size(); ++axis) {
    const float previous_acceleration =
        has_previous_sample_ ? previous_acceleration_ned_m_s2_[axis]
                             : acceleration_excitation_ned_m_s2[axis];
    const float trapezoidal_increment =
        0.5F * (previous_acceleration + acceleration_excitation_ned_m_s2[axis]) * dt_s;
    velocity_ned_m_s_[axis] =
        std::clamp(leak * velocity_ned_m_s_[axis] + trapezoidal_increment,
                   -parameters_.max_abs_velocity_m_s, parameters_.max_abs_velocity_m_s);
  }

  previous_acceleration_ned_m_s2_ = acceleration_excitation_ned_m_s2;
  has_previous_sample_ = true;
  return velocity_ned_m_s_;
}

}  // namespace px4_frequency_sweep
