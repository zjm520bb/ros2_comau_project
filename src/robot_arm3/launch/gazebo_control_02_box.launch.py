import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")
    base_launch_file = os.path.join(
        package_share_dir,
        "launch",
        "gazebo_control_02.launch.py",
    )
    trough_sdf_file = os.path.join(
        package_share_dir,
        "urdf",
        "grinding_trough_bbox.sdf",
    )

    if not os.path.exists(base_launch_file):
        raise FileNotFoundError(
            "Required base launch file was not found: "
            f"{base_launch_file}"
        )

    if not os.path.exists(trough_sdf_file):
        raise FileNotFoundError(
            "Required grinding trough Bounding Box SDF was not found: "
            f"{trough_sdf_file}"
        )

    world = LaunchConfiguration("world")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )

    spawn_grinding_trough_bbox = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_grinding_trough_bbox",
        output="screen",
        arguments=[
            "-world",
            world,
            "-file",
            trough_sdf_file,
            "-name",
            "grinding_trough_bbox",
            "-x",
            "3.6",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    delayed_spawn_grinding_trough_bbox = TimerAction(
        period=5.0,
        actions=[spawn_grinding_trough_bbox],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="default",
                description="Gazebo Sim world name used when spawning the Bounding Box.",
            ),
            base_launch,
            delayed_spawn_grinding_trough_bbox,
        ]
    )
