#include <exception>
#include <memory>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_frequency_sweep/frequency_sweep_mode.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  int return_code = 0;
  try {
    using FrequencySweepNode =
        px4_ros2::NodeWithMode<px4_frequency_sweep::FrequencySweepMode>;
    rclcpp::spin(std::make_shared<FrequencySweepNode>("frequency_sweep_mode", true));
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("frequency_sweep_mode"), "Fatal error: %s", exception.what());
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
