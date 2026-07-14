from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    cmd_port = LaunchConfiguration("cmd_port")
    control_port = LaunchConfiguration("control_port")
    enable_motion_control = LaunchConfiguration(
        "enable_motion_control"
    )
    enable_path_protocol = LaunchConfiguration(
        "enable_path_protocol"
    )
    c4g_protocol_version = LaunchConfiguration(
        "c4g_protocol_version"
    )
    feedback_port = LaunchConfiguration(
        "feedback_port"
    )
    joint_state_topic = LaunchConfiguration(
        "joint_state_topic"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="130.149.138.38",
                description="IP address of the C4G controller",
            ),
            DeclareLaunchArgument(
                "cmd_port",
                default_value="8000",
                description="C4G motion-command TCP port",
            ),
            DeclareLaunchArgument(
                "control_port",
                default_value="8002",
                description="C4G pause/resume/abort TCP port",
            ),
            DeclareLaunchArgument(
                "enable_motion_control",
                default_value="false",
                description=(
                    "Start the optional C4G pause/resume/abort "
                    "control client"
                ),
            ),
            DeclareLaunchArgument(
                "enable_path_protocol",
                default_value="false",
                description=(
                    "Enable the version-2 C4G PATH upload protocol"
                ),
            ),
            DeclareLaunchArgument(
                "c4g_protocol_version",
                default_value="1",
                description=(
                    "Explicit C4G protocol version; use 2 only "
                    "after deploying the PATH-capable PDL programs"
                ),
            ),
            DeclareLaunchArgument(
                "feedback_port",
                default_value="8001",
                description="C4G joint-feedback TCP port",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/joint_states",
                description=(
                    "ROS topic used by the C4G joint-feedback "
                    "publisher"
                ),
            ),
            Node(
                package="arm_tcp_bridge",
                executable="action_server",
                name="arm_execute_action_server",
                output="screen",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "cmd_port": cmd_port,
                        "control_port": control_port,
                        "enable_motion_control": (
                            enable_motion_control
                        ),
                        "enable_path_protocol": (
                            enable_path_protocol
                        ),
                        "c4g_protocol_version": (
                            c4g_protocol_version
                        ),
                        "path_action_name": "arm/execute_path",
                        "max_path_nodes": 1000,
                        "path_upload_timeout_s": 10.0,
                        "path_execution_timeout_s": 3600.0,
                        "connect_attempts": 10,
                        "connect_backoff_s": 0.5,
                        "connect_timeout_s": 3.0,
                        "io_timeout_s": 0.5,
                        "handshake_timeout_s": 3.0,
                        "final_timeout_s": 120.0,
                        "strict_echo_check": True,
                        "max_command_bytes": 254,
                        "tcp_debug": False,
                    }
                ],
            ),
            Node(
                package="arm_tcp_bridge",
                executable="motion_control_node",
                name="comau_motion_control",
                output="screen",
                condition=IfCondition(
                    enable_motion_control
                ),
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "control_port": control_port,
                        "connect_attempts": 10,
                        "connect_backoff_s": 0.5,
                        "connect_timeout_s": 3.0,
                        "io_timeout_s": 0.5,
                        "response_timeout_s": 3.0,
                        "max_command_bytes": 254,
                        "tcp_debug": False,
                        "service_prefix": "arm",
                    }
                ],
            ),
            Node(
                package="arm_tcp_bridge",
                executable="path_sequence_server",
                name="path_sequence_action_server",
                output="screen",
                condition=IfCondition(enable_path_protocol),
                parameters=[
                    {
                        "action_name": "/arm/execute_path_sequence",
                        "path_action_name": "/arm/execute_path",
                        "signal_sequence_service": "/arm/signal_sequence",
                        "joint_feedback_topic": joint_state_topic,
                        "default_feedback_timeout_s": 0.5,
                        "path_server_wait_timeout_s": 10.0,
                    }
                ],
            ),
            Node(
                package="arm_tcp_bridge",
                executable="joint_feedback_node",
                name="comau_joint_feedback",
                output="screen",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "feedback_port": feedback_port,
                        "joint_state_topic": joint_state_topic,
                        "connect_timeout_s": 3.0,
                        "io_timeout_s": 1.0,
                        "reconnect_backoff_s": 1.0,
                        "tcp_debug": False,
                        "joint_names": [
                            "joint_1",
                            "joint_2",
                            "joint_3",
                            "joint_4",
                            "joint_5",
                            "joint_6",
                        ],
                    }
                ],
            ),
        ]
    )
