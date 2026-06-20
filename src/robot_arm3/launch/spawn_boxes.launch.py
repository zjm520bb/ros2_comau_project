import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_bbox(name, sdf_file, world, x, y, z="0.0"):
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        arguments=[
            "-world",
            world,
            "-file",
            sdf_file,
            "-name",
            name,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
        ],
    )


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")

    bbox_files = {
        "box1": os.path.join(package_share_dir, "urdf", "bbox_box1.sdf"),
        "box2": os.path.join(package_share_dir, "urdf", "bbox_box2.sdf"),
        "box3": os.path.join(package_share_dir, "urdf", "bbox_box3.sdf"),
        "box4": os.path.join(package_share_dir, "urdf", "bbox_box4.sdf"),
        "box5": os.path.join(package_share_dir, "urdf", "bbox_box5.sdf"),
    }

    missing_paths = [
        path for path in bbox_files.values()
        if not os.path.exists(path)
    ]
    if missing_paths:
        raise FileNotFoundError(
            "Required Gazebo Bounding Box model file(s) "
            "were not found: "
            + ", ".join(missing_paths)
        )

    world = LaunchConfiguration("world")
    spawn_delay_s = LaunchConfiguration("spawn_delay_s")

    spawn_bboxes = [
        _spawn_bbox("box1_a", bbox_files["box1"], world, "-2.04", "-0.58"),
        _spawn_bbox("box1_b", bbox_files["box1"], world, "-2.04", "0.58"),
        _spawn_bbox("box5", bbox_files["box5"], world, "2.5", "-2.5"),
        _spawn_bbox("box2", bbox_files["box2"], world, "-0.525", "2.35"),
        _spawn_bbox("box3", bbox_files["box3"], world, "2.0", "2.5"),
        _spawn_bbox("box4", bbox_files["box4"], world, "0.0", "-2.54"),
    ]

    delayed_spawn_bboxes = TimerAction(
        period=spawn_delay_s,
        actions=spawn_bboxes,
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
                "spawn_delay_s",
                default_value="0.0",
                description="Seconds to wait before spawning Bounding Boxes.",
            ),
            delayed_spawn_bboxes,
        ]
    )
