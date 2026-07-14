import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("robot_arm3_moveit_config")
    bridge_share = get_package_share_directory("arm_tcp_bridge")
    start_c4g_bridge = LaunchConfiguration("start_c4g_bridge")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share,
                "launch",
                "gazebo_moveit_08.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    c4g_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                "launch",
                "arm_tcp_bridge.launch.py",
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "enable_motion_control": "true",
            "enable_path_protocol": "true",
            "c4g_protocol_version": "2",
            "joint_state_topic": "/c4g/joint_states",
        }.items(),
        condition=IfCondition(start_c4g_bridge),
    )
    manager = Node(
        package="arm_tcp_bridge",
        executable="offline_sequence_manager",
        name="offline_path_sequence_manager",
        output="screen",
        parameters=[
            {
                "prepared_path_topic": "/offline/prepared_path",
                "sequence_action_name": "/arm/execute_path_sequence",
                "export_path": LaunchConfiguration("export_path"),
                "start_tolerance_deg": LaunchConfiguration(
                    "start_tolerance_deg"
                ),
            }
        ],
    )
    simulation_sequence_server = Node(
        package="arm_tcp_bridge",
        executable="path_sequence_server",
        name="simulation_path_sequence_action_server",
        output="screen",
        parameters=[
            {
                "action_name": "/sim/arm/execute_path_sequence",
                "path_action_name": "/sim/arm/execute_path",
                "signal_sequence_service": "/sim/arm/signal_sequence",
                "joint_feedback_topic": "/joint_states",
            }
        ],
    )
    c4g_gazebo_sync = Node(
        package="robot_arm3",
        executable="c4g_gazebo_sync_node.py",
        name="c4g_gazebo_sync",
        output="screen",
        condition=IfCondition(start_c4g_bridge),
        parameters=[
            {
                "enabled_on_start": False,
                "initial_sync_mode": "teleport",
                # Final preview leaves Gazebo at the PATH end pose.  When
                # /send enables the mirror, teleport directly to C4G instead
                # of waiting for the old Gazebo startup pose again.
                "require_initial_gazebo_pose": False,
                "startup_delay_s": 0.0,
                "c4g_joint_states_topic": "/c4g/joint_states",
                "gazebo_joint_states_topic": "/joint_states",
                "trajectory_topic": "/arm_controller/joint_trajectory",
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_c4g_bridge", default_value="false"),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="130.149.138.38",
            ),
            DeclareLaunchArgument(
                "export_path",
                default_value="/tmp/prepared_path.c4gseq.yaml",
            ),
            DeclareLaunchArgument(
                "start_tolerance_deg",
                default_value="0.5",
            ),
            simulation,
            c4g_bridge,
            simulation_sequence_server,
            c4g_gazebo_sync,
            manager,
        ]
    )
