#pragma once

#include <array>

#include "px4_frequency_sweep/parameters.hpp"

namespace px4_frequency_sweep {

struct SweepSample {
  float value{0.F};
  float instantaneous_frequency_hz{0.F};
  float envelope{0.F};
  bool finished{false};
};

class SweepGenerator {
 public:
  explicit SweepGenerator(SweepParameters parameters);

  SweepSample sample(float elapsed_s) const;

 private:
  float phaseRad(float elapsed_s) const;
  float instantaneousFrequencyHz(float elapsed_s) const;
  float envelope(float elapsed_s) const;

  SweepParameters parameters_;
};

class LeakyVelocityIntegrator {
 public:
  explicit LeakyVelocityIntegrator(VelocityIntegratorParameters parameters);

  void reset();
  const std::array<float, 3>& update(const std::array<float, 3>& acceleration_excitation_ned_m_s2,
                                     float dt_s, float sweep_elapsed_s);
  const std::array<float, 3>& velocityNed() const { return velocity_ned_m_s_; }

 private:
  VelocityIntegratorParameters parameters_;
  std::array<float, 3> velocity_ned_m_s_{};
  std::array<float, 3> previous_acceleration_ned_m_s2_{};
  bool has_previous_sample_{false};
};

}  // namespace px4_frequency_sweep
