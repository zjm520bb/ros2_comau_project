from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    cmd_port = LaunchConfiguration("cmd_port")
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
