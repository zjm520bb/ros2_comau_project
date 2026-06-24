import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_arm3_peripherals")
    default_config = os.path.join(
        package_share,
        "config",
        "peripherals.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Peripheral simulation configuration file.",
            ),
            Node(
                package="robot_arm3_peripherals",
                executable="peripheral_sim_node",
                name="peripheral_sim_node",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                    }
                ],
            ),
        ]
    )
