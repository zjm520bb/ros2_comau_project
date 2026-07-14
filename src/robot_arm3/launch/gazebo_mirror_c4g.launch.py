import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")
    package_share_parent = os.path.dirname(package_share_dir)
    arm_tcp_bridge_share_dir = get_package_share_directory("arm_tcp_bridge")
    ros_gz_sim_share_dir = get_package_share_directory("ros_gz_sim")
    xacro_file = os.path.join(
        package_share_dir,
        "urdf",
        "robot_arm3_control_05.urdf.xacro",
    )
    controller_config_file = os.path.join(
        package_share_dir,
        "config",
        "ros2_controllers_mirror_c4g.yaml",
    )

    robot_ip = LaunchConfiguration("robot_ip")
    cmd_port = LaunchConfiguration("cmd_port")
    control_port = LaunchConfiguration("control_port")
    feedback_port = LaunchConfiguration("feedback_port")
    enable_motion_control = LaunchConfiguration(
        "enable_motion_control"
    )
    enable_path_protocol = LaunchConfiguration(
        "enable_path_protocol"
    )
    c4g_protocol_version = LaunchConfiguration(
        "c4g_protocol_version"
    )
    c4g_joint_states_topic = LaunchConfiguration(
        "c4g_joint_states_topic"
    )
    start_arm_tcp_bridge = LaunchConfiguration(
        "start_arm_tcp_bridge"
    )
    initial_sync_mode = LaunchConfiguration("initial_sync_mode")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " controller_config:=",
                controller_config_file,
            ]
        ),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share_dir,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    arm_tcp_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                arm_tcp_bridge_share_dir,
                "launch",
                "arm_tcp_bridge.launch.py",
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "cmd_port": cmd_port,
            "control_port": control_port,
            "feedback_port": feedback_port,
            "enable_motion_control": enable_motion_control,
            "enable_path_protocol": enable_path_protocol,
            "c4g_protocol_version": c4g_protocol_version,
            "joint_state_topic": c4g_joint_states_topic,
        }.items(),
        condition=IfCondition(start_arm_tcp_bridge),
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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_arm3_mirror_c4g",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-topic",
            "/robot_description",
            "-name",
            "robot_arm3_mirror_c4g",
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    internal_passive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "internal_passive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    gazebo_initial_hold_publisher = Node(
        package="robot_arm3",
        executable="gazebo_initial_hold_publisher_06.py",
        name="gazebo_initial_hold_publisher_06",
        output="screen",
    )

    gazebo_passive_joint_controller = Node(
        package="robot_arm3",
        executable="gazebo_passive_joint_controller_06.py",
        name="gazebo_passive_joint_controller_06",
        output="screen",
    )

    c4g_gazebo_sync = Node(
        package="robot_arm3",
        executable="c4g_gazebo_sync_node.py",
        name="c4g_gazebo_sync",
        output="screen",
        parameters=[
            {
                "c4g_joint_states_topic": c4g_joint_states_topic,
                "gazebo_joint_states_topic": "/joint_states",
                "trajectory_topic": "/arm_controller/joint_trajectory",
                "startup_delay_s": 0.5,
                "initial_gazebo_positions": [
                    0.0,
                    0.0,
                    -1.5708,
                    0.0,
                    0.0,
                    0.0,
                ],
                "initial_pose_tolerance_rad": 0.01,
                "initial_pose_stable_duration_s": 0.5,
                "initial_pose_timeout_s": 15.0,
                "initial_sync_mode": initial_sync_mode,
                "mirror_joint_reset_service": (
                    "/gazebo/reset_robot_joints_for_mirror"
                ),
                "passive_command_topic": (
                    "/internal_passive_controller/commands"
                ),
                "passive_solver_enable_service": (
                    "/gazebo_passive_joint_controller_06/set_enabled"
                ),
                "require_passive_solver_handover": True,
                "active_joint_lower_limits": [
                    -3.14, -1.3, -4.0317, -47.12, -2.18, -47.12,
                ],
                "active_joint_upper_limits": [
                    3.14, 1.3, 0.0, 47.12, 2.18, 47.12,
                ],
                "passive_joint_lower_limits": [-3.14, -3.14],
                "passive_joint_upper_limits": [3.14, 3.14],
                "teleport_limit_tolerance_rad": 0.001,
                "joint_reset_timeout_s": 20.0,
                "reset_tolerance_rad": 0.01,
                "reset_velocity_tolerance_rad_s": 0.02,
                "reset_stable_duration_s": 0.2,
                "coupling_offset": 1.5708,
                "blend_duration_s": 2.0,
                "live_time_from_start_s": 0.08,
                "feedback_timeout_s": 0.5,
                "command_publish_rate_hz": 30.0,
                "live_log_change_threshold_rad": 0.001,
                "live_log_heartbeat_s": 20.0,
                "joint_names": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ],
                "passive_joint_names": [
                    "joint_7",
                    "joint_8",
                ],
                "signs": [
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                ],
                "offsets": [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            }
        ],
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            internal_passive_controller_spawner,
        ],
    )

    start_initial_hold_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                gazebo_initial_hold_publisher,
            ],
        )
    )

    start_sync_after_initial_hold_command = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_initial_hold_publisher,
            on_exit=[
                gazebo_passive_joint_controller,
                c4g_gazebo_sync,
            ],
        )
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
                "feedback_port",
                default_value="8001",
                description="C4G joint-feedback TCP port",
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
            DeclareLaunchArgument(
                "c4g_joint_states_topic",
                default_value="/c4g/joint_states",
                description="Topic used for C4G joint feedback.",
            ),
            DeclareLaunchArgument(
                "start_arm_tcp_bridge",
                default_value="true",
                description=(
                    "Start arm_tcp_bridge for real C4G command and "
                    "feedback."
                ),
            ),
            DeclareLaunchArgument(
                "initial_sync_mode",
                default_value="blend",
                description=(
                    "Initial Gazebo synchronization: blend uses a trajectory "
                    "transition by default; explicitly select teleport for "
                    "an instantaneous joint reset."
                ),
            ),
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
            arm_tcp_bridge,
            robot_state_publisher,
            spawn_robot,
            delayed_spawners,
            start_initial_hold_after_arm_controller,
            start_sync_after_initial_hold_command,
        ]
    )
