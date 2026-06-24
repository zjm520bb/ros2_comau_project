import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")
    xacro_file = os.path.join(package_share_dir, "urdf", "robot_arm3_control_03.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    passive_joint_solver = Node(
        package="robot_arm3",
        executable="passive_joint_solver_node_03.py",
        name="passive_joint_solver_03",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher_gui,
            passive_joint_solver,
            rviz2,
        ]
    )
