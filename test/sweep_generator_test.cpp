#include <cmath>
#include <cstdlib>

#include "px4_frequency_sweep/sweep_generator.hpp"

namespace {

bool near(float lhs, float rhs, float tolerance = 1e-4F)
{
  return std::abs(lhs - rhs) <= tolerance;
}

void require(bool condition)
{
  if (!condition) {
    std::abort();
  }
}

void testLinearSweep()
{
  px4_frequency_sweep::SweepParameters parameters;
  parameters.waveform = px4_frequency_sweep::Waveform::Linear;
  parameters.start_frequency_hz = 1.F;
  parameters.end_frequency_hz = 5.F;
  parameters.duration_s = 4.F;
  parameters.amplitude = 2.F;
  parameters.fade_in_s = 0.F;
  parameters.fade_out_s = 0.F;

  const px4_frequency_sweep::SweepGenerator generator(parameters);
  require(near(generator.sample(0.F).instantaneous_frequency_hz, 1.F));
  require(near(generator.sample(2.F).instantaneous_frequency_hz, 3.F));
  require(near(generator.sample(4.F).instantaneous_frequency_hz, 5.F));
  require(generator.sample(4.F).finished);
}

void testFadeEnvelope()
{
  px4_frequency_sweep::SweepParameters parameters;
  parameters.start_frequency_hz = 1.F;
  parameters.end_frequency_hz = 1.F;
  parameters.duration_s = 10.F;
  parameters.fade_in_s = 2.F;
  parameters.fade_out_s = 2.F;

  const px4_frequency_sweep::SweepGenerator generator(parameters);
  require(near(generator.sample(0.F).envelope, 0.F));
  require(near(generator.sample(1.F).envelope, 0.5F));
  require(near(generator.sample(5.F).envelope, 1.F));
  require(near(generator.sample(10.F).envelope, 0.F));
}

void testLogarithmicSweep()
{
  px4_frequency_sweep::SweepParameters parameters;
  parameters.waveform = px4_frequency_sweep::Waveform::Logarithmic;
  parameters.start_frequency_hz = 1.F;
  parameters.end_frequency_hz = 16.F;
  parameters.duration_s = 8.F;
  parameters.fade_in_s = 0.F;
  parameters.fade_out_s = 0.F;

  const px4_frequency_sweep::SweepGenerator generator(parameters);
  require(near(generator.sample(0.F).instantaneous_frequency_hz, 1.F));
  require(near(generator.sample(4.F).instantaneous_frequency_hz, 4.F));
  require(near(generator.sample(8.F).instantaneous_frequency_hz, 16.F));
}

void testLeakyVelocityIntegrator()
{
  px4_frequency_sweep::VelocityIntegratorParameters parameters;
  parameters.enabled = true;
  parameters.start_delay_s = 0.F;
  parameters.leak_time_constant_s = 1000.F;
  parameters.max_abs_velocity_m_s = 10.F;

  px4_frequency_sweep::LeakyVelocityIntegrator integrator(parameters);
  integrator.update({0.F, 1.F, 0.F}, 0.1F, 0.F);
  const auto& velocity = integrator.update({0.F, 1.F, 0.F}, 0.1F, 0.1F);
  require(near(velocity[0], 0.F));
  require(velocity[1] > 0.19F && velocity[1] < 0.21F);
  require(near(velocity[2], 0.F));
}

float integrateConstantAcceleration(float dt_s)
{
  px4_frequency_sweep::VelocityIntegratorParameters parameters;
  parameters.enabled = true;
  parameters.start_delay_s = 0.F;
  parameters.leak_time_constant_s = 2.F;
  parameters.max_abs_velocity_m_s = 10.F;

  px4_frequency_sweep::LeakyVelocityIntegrator integrator(parameters);
  float elapsed_s = 0.F;
  while (elapsed_s < 1.F - 0.5F * dt_s) {
    integrator.update({1.F, 0.F, 0.F}, dt_s, elapsed_s);
    elapsed_s += dt_s;
  }
  return integrator.velocityNed()[0];
}

void testIntegratorIsRateIndependent()
{
  const float velocity_100_hz = integrateConstantAcceleration(0.01F);
  const float velocity_200_hz = integrateConstantAcceleration(0.005F);
  require(std::abs(velocity_100_hz - velocity_200_hz) < 0.005F);
}

}  // namespace

int main()
{
  testLinearSweep();
  testFadeEnvelope();
  testLogarithmicSweep();
  testLeakyVelocityIntegrator();
  testIntegratorIsRateIndependent();
  return 0;
}
