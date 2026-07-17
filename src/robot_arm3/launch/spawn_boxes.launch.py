import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_bbox(name, sdf_file, world, pose):
    x, y, z, roll, pitch, yaw = [str(value) for value in pose]
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
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
    )


def _load_boxes(config_file, package_share_dir):
    with open(config_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("boxes"), list):
        raise ValueError("Environment config must contain a 'boxes' list")

    c4g_base_height = float(config.get("gazebo_c4g_base_height", 0.0))
    if not math.isfinite(c4g_base_height):
        raise ValueError("gazebo_c4g_base_height must be finite")

    boxes = []
    identifiers = set()
    for entry in config["boxes"]:
        if not isinstance(entry, dict):
            raise ValueError("Each environment box must be a mapping")
        identifier = str(entry.get("id", "")).strip()
        if not identifier or identifier in identifiers:
            raise ValueError(f"Invalid or duplicate environment box id: {identifier!r}")
        identifiers.add(identifier)

        pose = entry.get("pose")
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError(f"Environment box {identifier!r} requires a six-value pose")
        sdf_file = os.path.join(package_share_dir, "urdf", str(entry.get("sdf", "")))
        if not os.path.isfile(sdf_file):
            raise FileNotFoundError(f"Bounding Box SDF was not found: {sdf_file}")
        gazebo_pose = list(pose)
        gazebo_pose[2] += c4g_base_height
        boxes.append((identifier, sdf_file, gazebo_pose))
    return boxes


def _launch_setup(context):
    package_share_dir = get_package_share_directory("robot_arm3")
    config_file = LaunchConfiguration("environment_config").perform(context)
    boxes = _load_boxes(config_file, package_share_dir)
    world = LaunchConfiguration("world").perform(context)
    spawn_delay_s = float(
        LaunchConfiguration("spawn_delay_s").perform(context)
    )
    spawn_interval_s = float(
        LaunchConfiguration("spawn_interval_s").perform(context)
    )
    if not math.isfinite(spawn_delay_s) or spawn_delay_s < 0.0:
        raise ValueError("spawn_delay_s must be finite and non-negative")
    if not math.isfinite(spawn_interval_s) or spawn_interval_s <= 0.0:
        raise ValueError("spawn_interval_s must be finite and greater than zero")

    return [
        TimerAction(
            period=spawn_delay_s + index * spawn_interval_s,
            actions=[_spawn_bbox(identifier, sdf_file, world, pose)],
        )
        for index, (identifier, sdf_file, pose) in enumerate(boxes)
    ]


def generate_launch_description():
    package_share_dir = get_package_share_directory("robot_arm3")
    default_config = os.path.join(
        package_share_dir,
        "config",
        "environment_boxes.yaml",
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
            DeclareLaunchArgument(
                "spawn_interval_s",
                default_value="1.5",
                description="Seconds between consecutive Bounding Box spawns.",
            ),
            DeclareLaunchArgument(
                "environment_config",
                default_value=default_config,
                description="Shared Gazebo and MoveIt environment box configuration.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
