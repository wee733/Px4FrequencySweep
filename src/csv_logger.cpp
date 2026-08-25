#include "px4_frequency_sweep/csv_logger.hpp"

#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

namespace px4_frequency_sweep {
namespace {

std::string timestampForFilename()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  const std::tm local_time = *std::localtime(&time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << std::setw(3)
         << std::setfill('0') << milliseconds.count();
  return stream.str();
}

template <typename T>
void writeArray(std::ostream& stream, const std::array<T, 3>& values)
{
  stream << '[' << values[0] << ',' << values[1] << ',' << values[2] << ']';
}

}  // namespace

CsvLogger::~CsvLogger()
{
  close();
}

bool CsvLogger::open(const FrequencySweepParameters& parameters,
                     const std::array<float, 3>& reference_position_ned_m,
                     float reference_yaw_ned_rad, std::string& error)
{
  close();

  try {
    std::filesystem::create_directories(parameters.logging.directory);
    const std::string filename =
        "frequency_sweep_" + toString(parameters.sweep.target) + "_" +
        timestampForFilename() + ".csv";
    path_ = (std::filesystem::path(parameters.logging.directory) / filename).string();
    stream_.open(path_, std::ios::out | std::ios::trunc);
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }

  if (!stream_.is_open()) {
    error = "Failed to open '" + path_ + "'";
    return false;
  }

  flush_every_n_samples_ = parameters.logging.flush_every_n_samples;
  samples_since_flush_ = 0;
  stream_ << std::setprecision(9);
  stream_ << "# mode_name=" << parameters.mode.name << '\n';
  stream_ << "# update_rate_hz=" << parameters.mode.update_rate_hz << '\n';
  stream_ << "# target=" << toString(parameters.sweep.target) << '\n';
  stream_ << "# horizontal_frame=" << toString(parameters.sweep.horizontal_frame) << '\n';
  stream_ << "# waveform=" << toString(parameters.sweep.waveform) << '\n';
  stream_ << "# start_frequency_hz=" << parameters.sweep.start_frequency_hz << '\n';
  stream_ << "# end_frequency_hz=" << parameters.sweep.end_frequency_hz << '\n';
  stream_ << "# amplitude=" << parameters.sweep.amplitude << '\n';
  stream_ << "# duration_s=" << parameters.sweep.duration_s << '\n';
  stream_ << "# repetitions=" << parameters.sweep.repetitions << '\n';
  stream_ << "# reference_position_ned_m=";
  writeArray(stream_, reference_position_ned_m);
  stream_ << '\n';
  stream_ << "# reference_yaw_ned_rad=" << reference_yaw_ned_rad << '\n';
  stream_ << "ros_time_s,phase,repetition,phase_elapsed_s,sweep_frequency_hz,sweep_envelope,"
             "sweep_value,sp_position_x,sp_position_y,sp_position_z,sp_velocity_x,"
             "sp_velocity_y,sp_velocity_z,sp_acceleration_x,sp_acceleration_y,"
             "sp_acceleration_z,sp_yaw,sp_yaw_rate,position_x,position_y,position_z,"
             "velocity_x,velocity_y,velocity_z,roll,pitch,yaw,angular_velocity_x,"
             "angular_velocity_y,angular_velocity_z\n";
  stream_.flush();
  return true;
}

void CsvLogger::write(double ros_time_s, ModePhase phase, int repetition_index,
                      float phase_elapsed_s, const SweepSample& sweep,
                      const TrajectoryCommand& command, const TelemetrySnapshot& telemetry)
{
  if (!stream_.is_open()) {
    return;
  }

  stream_ << ros_time_s << ',' << toString(phase) << ',' << repetition_index + 1 << ','
          << phase_elapsed_s << ',' << sweep.instantaneous_frequency_hz << ',' << sweep.envelope
          << ',' << sweep.value;
  for (const auto& value : command.position_ned_m) stream_ << ',' << optionalOrNan(value);
  for (const auto& value : command.velocity_ned_m_s) stream_ << ',' << optionalOrNan(value);
  for (const auto& value : command.acceleration_ned_m_s2) stream_ << ',' << optionalOrNan(value);
  stream_ << ',' << optionalOrNan(command.yaw_ned_rad) << ','
          << optionalOrNan(command.yaw_rate_ned_rad_s);
  for (float value : telemetry.position_ned_m) stream_ << ',' << value;
  for (float value : telemetry.velocity_ned_m_s) stream_ << ',' << value;
  for (float value : telemetry.attitude_rpy_rad) stream_ << ',' << value;
  for (float value : telemetry.angular_velocity_frd_rad_s) stream_ << ',' << value;
  stream_ << '\n';

  if (++samples_since_flush_ >= flush_every_n_samples_) {
    stream_.flush();
    samples_since_flush_ = 0;
  }
}

void CsvLogger::close()
{
  if (stream_.is_open()) {
    stream_.flush();
    stream_.close();
  }
  samples_since_flush_ = 0;
}

float CsvLogger::optionalOrNan(const std::optional<float>& value)
{
  return value.value_or(std::numeric_limits<float>::quiet_NaN());
}

}  // namespace px4_frequency_sweep
