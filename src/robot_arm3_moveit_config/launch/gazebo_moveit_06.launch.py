import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_arm3_share = get_package_share_directory("robot_arm3")
    moveit_config_share = get_package_share_directory(
        "robot_arm3_moveit_config"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "robot_arm3",
            package_name="robot_arm3_moveit_config",
        )
        .trajectory_execution(
            file_path="config/moveit_gazebo_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_arm3_share,
                "launch",
                "gazebo_control_06.launch.py",
            )
        )
    )

    trajectory_bridge = Node(
        package="robot_arm3",
        executable="moveit_gazebo_trajectory_bridge.py",
        name="moveit_gazebo_trajectory_bridge",
        output="screen",
        parameters=[
            {
                "input_action_name": (
                    "/moveit_arm_controller/follow_joint_trajectory"
                ),
                "output_action_name": (
                    "/arm_controller/follow_joint_trajectory"
                ),
                "coupling_offset": 1.5708,
                "joint_7_min": -1.151917306,
                "joint_7_max": 1.047197551,
                "joint_3_min": -4.0317,
                "joint_3_max": 0.0,
                "joint_3_max_velocity": 1.0,
                "downstream_wait_timeout_s": 10.0,
                "use_sim_time": True,
            }
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_trajectory_execution": True,
                "use_sim_time": True,
                # Gazebo's robot_state_publisher owns /robot_description.
                # Publishing MoveIt's model there can make Gazebo spawn the
                # description without the gz_ros2_control plugin.
                "publish_robot_description": False,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="log",
        arguments=[
            "-d",
            os.path.join(moveit_config_share, "config", "moveit.rviz"),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start MoveIt RViz alongside Gazebo.",
            ),
            gazebo,
            clock_bridge,
            trajectory_bridge,
            move_group,
            rviz,
        ]
    )
