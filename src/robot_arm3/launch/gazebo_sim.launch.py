import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")
    package_share_parent = os.path.dirname(package_share_dir)
    ros_gz_sim_share_dir = get_package_share_directory("ros_gz_sim")
    urdf_file = os.path.join(package_share_dir, "urdf", "robot_arm3_gazebo.urdf")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            # Start paused so the model is spawned before physics begins.
            "gz_args": "empty.sdf",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_arm3",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-file",
            urdf_file,
            "-name",
            "robot_arm3",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=package_share_parent,
                separator=os.pathsep,
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=package_share_parent,
                separator=os.pathsep,
            ),
            gz_sim,
            spawn_robot,
        ]
    )
