import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _load_topics(config_file):
    with open(config_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    topics = config.get("recording", {}).get("topics", [])
    if not isinstance(topics, list) or not topics:
        raise ValueError("Recording config must contain a non-empty topics list")
    return [str(topic) for topic in topics]


def _launch_setup(context):
    config_file = LaunchConfiguration("topics_file").perform(context)
    output_root = LaunchConfiguration("output_root").perform(context)
    storage_id = LaunchConfiguration("storage_id").perform(context)
    output_name = LaunchConfiguration("output_name").perform(context)

    if output_name == "auto":
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_name = f"real_{timestamp}"

    output_dir = os.path.join(output_root, output_name)
    topics = _load_topics(config_file)

    return [
        LogInfo(msg=f"Recording real rosbag to: {output_dir}"),
        LogInfo(msg="Recording topics: " + ", ".join(topics)),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                output_dir,
                "--storage",
                storage_id,
                *topics,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("robot_arm3_data_recording")
    default_topics = os.path.join(
        package_share,
        "config",
        "record_real_topics.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topics_file",
                default_value=default_topics,
                description="YAML file listing real-system topics to record.",
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value="/tmp/robot_arm3_bags",
                description="Directory where bag runs are written.",
            ),
            DeclareLaunchArgument(
                "output_name",
                default_value="auto",
                description="Bag directory name, or 'auto' for timestamped output.",
            ),
            DeclareLaunchArgument(
                "storage_id",
                default_value="sqlite3",
                description="rosbag2 storage plugin id.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
