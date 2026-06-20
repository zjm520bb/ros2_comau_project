import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")

    base_launch_file = os.path.join(
        package_share_dir,
        "launch",
        "gazebo_control_06.launch.py",
    )
    spawn_boxes_launch_file = os.path.join(
        package_share_dir,
        "launch",
        "spawn_boxes.launch.py",
    )

    missing_paths = [
        path for path in [base_launch_file, spawn_boxes_launch_file]
        if not os.path.exists(path)
    ]
    if missing_paths:
        raise FileNotFoundError(
            "Required launch file(s) were not found: "
            + ", ".join(missing_paths)
        )

    world = LaunchConfiguration("world")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )

    spawn_boxes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_boxes_launch_file),
        launch_arguments={
            "world": world,
            "spawn_delay_s": "5.0",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="empty",
                description=(
                    "Gazebo Sim world name used when spawning "
                    "Bounding Boxes."
                ),
            ),
            base_launch,
            spawn_boxes,
        ]
    )
