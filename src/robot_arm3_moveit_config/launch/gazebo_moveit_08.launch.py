import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = get_package_share_directory("robot_arm3")
    moveit_share = get_package_share_directory("robot_arm3_moveit_config")
    environment_config = LaunchConfiguration("environment_config")

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "gazebo_moveit_07.launch.py")
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    gazebo_boxes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, "launch", "spawn_boxes.launch.py")
        ),
        launch_arguments={
            "world": "empty",
            "spawn_delay_s": "5.0",
            "spawn_interval_s": "1.5",
            "environment_config": environment_config,
        }.items(),
    )

    moveit_boxes = Node(
        package="robot_arm3",
        executable="moveit_environment_boxes.py",
        name="moveit_environment_boxes",
        output="screen",
        parameters=[
            {
                "environment_config": environment_config,
                "sdf_directory": os.path.join(robot_share, "urdf"),
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start MoveIt RViz alongside Gazebo.",
            ),
            DeclareLaunchArgument(
                "environment_config",
                default_value=os.path.join(
                    robot_share,
                    "config",
                    "environment_boxes.yaml",
                ),
                description="Shared Gazebo and MoveIt environment configuration.",
            ),
            base_stack,
            gazebo_boxes,
            moveit_boxes,
        ]
    )
