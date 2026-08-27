#pragma once

#include <array>
#include <fstream>
#include <string>

#include "px4_frequency_sweep/configuration.hpp"
#include "px4_frequency_sweep/sweep_generator.hpp"
#include "px4_frequency_sweep/types.hpp"

namespace px4_frequency_sweep {

class CsvLogger {
 public:
  CsvLogger() = default;
  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;
  ~CsvLogger();

  bool open(const FrequencySweepParameters& parameters,
            const std::array<float, 3>& reference_position_ned_m, float reference_yaw_ned_rad,
            std::string& error);
  void write(double ros_time_s, float dt_s, ModePhase phase, const SweepStage& stage,
             int repetition_index, float phase_elapsed_s, const SweepSample& sweep,
             const TrajectoryCommand& command, const TelemetrySnapshot& telemetry);
  void close();

  bool isOpen() const { return stream_.is_open(); }
  const std::string& path() const { return path_; }

 private:
  static float optionalOrNan(const std::optional<float>& value);
  // ros_time_s is a Unix epoch value (~1.8e9). The stream runs at setprecision(9) for the float
  // columns, which would round it to 10-second granularity, so it needs fixed notation.
  void writeEpochSeconds(double seconds);

  std::ofstream stream_;
  std::string path_;
  int flush_every_n_samples_{100};
  int samples_since_flush_{0};
};

}  // namespace px4_frequency_sweep
