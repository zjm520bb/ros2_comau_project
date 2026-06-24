import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("robot_arm3_peripherals")
    peripherals_launch = os.path.join(
        package_share,
        "launch",
        "peripherals_sim.launch.py",
    )
    default_process = os.path.join(
        package_share,
        "config",
        "process_demo.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "process_file",
                default_value=default_process,
                description="Process sequence YAML file.",
            ),
            DeclareLaunchArgument(
                "arm_action_name",
                default_value="/sim/arm/execute",
                description="Robot command action used by the process.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="Start the process automatically after launch.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(peripherals_launch)
            ),
            Node(
                package="robot_arm3_peripherals",
                executable="process_orchestrator_node",
                name="process_orchestrator",
                output="screen",
                parameters=[
                    {
                        "process_file": LaunchConfiguration("process_file"),
                        "arm_action_name": LaunchConfiguration(
                            "arm_action_name"
                        ),
                        "auto_start": ParameterValue(
                            LaunchConfiguration("auto_start"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
