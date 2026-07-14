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
        "gazebo_mirror_c4g.launch.py",
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
    initial_sync_mode = LaunchConfiguration("initial_sync_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    cmd_port = LaunchConfiguration("cmd_port")
    control_port = LaunchConfiguration("control_port")
    feedback_port = LaunchConfiguration("feedback_port")
    c4g_joint_states_topic = LaunchConfiguration(
        "c4g_joint_states_topic"
    )
    start_arm_tcp_bridge = LaunchConfiguration(
        "start_arm_tcp_bridge"
    )
    enable_motion_control = LaunchConfiguration(
        "enable_motion_control"
    )
    enable_path_protocol = LaunchConfiguration(
        "enable_path_protocol"
    )
    c4g_protocol_version = LaunchConfiguration(
        "c4g_protocol_version"
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            "initial_sync_mode": initial_sync_mode,
            "robot_ip": robot_ip,
            "cmd_port": cmd_port,
            "control_port": control_port,
            "feedback_port": feedback_port,
            "c4g_joint_states_topic": c4g_joint_states_topic,
            "start_arm_tcp_bridge": start_arm_tcp_bridge,
            "enable_motion_control": enable_motion_control,
            "enable_path_protocol": enable_path_protocol,
            "c4g_protocol_version": c4g_protocol_version,
        }.items(),
    )

    spawn_boxes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_boxes_launch_file),
        launch_arguments={
            "world": world,
            "spawn_delay_s": "20.0",
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
            DeclareLaunchArgument(
                "initial_sync_mode",
                default_value="blend",
                description=(
                    "Use a trajectory transition by default in the Bounding "
                    "Box mirror world; explicitly select teleport for an "
                    "instantaneous joint reset."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="130.149.138.38",
                description="IP address of the C4G controller.",
            ),
            DeclareLaunchArgument(
                "cmd_port",
                default_value="8000",
                description="C4G motion-command TCP port.",
            ),
            DeclareLaunchArgument(
                "control_port",
                default_value="8002",
                description="C4G pause/resume/abort TCP port.",
            ),
            DeclareLaunchArgument(
                "feedback_port",
                default_value="8001",
                description="C4G joint-feedback TCP port.",
            ),
            DeclareLaunchArgument(
                "c4g_joint_states_topic",
                default_value="/c4g/joint_states",
                description="Topic used for C4G joint feedback.",
            ),
            DeclareLaunchArgument(
                "start_arm_tcp_bridge",
                default_value="true",
                description="Start the C4G TCP bridge.",
            ),
            DeclareLaunchArgument(
                "enable_motion_control",
                default_value="true",
                description=(
                    "Enable pause/resume/abort and PATH node-wait "
                    "control services."
                ),
            ),
            DeclareLaunchArgument(
                "enable_path_protocol",
                default_value="true",
                description=(
                    "Enable C4G PATH upload and PATH sequence execution."
                ),
            ),
            DeclareLaunchArgument(
                "c4g_protocol_version",
                default_value="2",
                description=(
                    "Use 2 only after deploying the PATH-capable "
                    "C4G PDL programs."
                ),
            ),
            base_launch,
            spawn_boxes,
        ]
    )
