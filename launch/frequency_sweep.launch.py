from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("px4_frequency_sweep"))
    default_config = str(package_share / "config" / "frequency_sweep.yaml")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Absolute path to a Frequency Sweep ROS 2 parameter YAML file",
            ),
            # Which vehicle's PX4 topics to use is set by 'px4_topic_namespace_prefix' in the
            # YAML. It is deliberately not a launch argument: an argument defaulting to "" would
            # override the YAML value, so a prefix set there would be silently dropped.
            # For a one-off override without editing the YAML:
            #   ros2 run px4_frequency_sweep frequency_sweep_mode --ros-args \
            #     --params-file <yaml> -p px4_topic_namespace_prefix:=/drone0
            Node(
                package="px4_frequency_sweep",
                executable="frequency_sweep_mode",
                name="frequency_sweep_mode",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
