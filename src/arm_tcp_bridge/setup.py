
import os
from glob import glob

from setuptools import find_packages, setup


package_name = "arm_tcp_bridge"


setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(
        exclude=["test"],
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            os.path.join(
                "share",
                package_name,
            ),
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Jiaming",
    maintainer_email="jm867019644@gmail.com",
    description=(
        "ROS 2 TCP bridge for Comau C4G motion commands "
        "and actual joint-position feedback"
    ),
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            (
                "action_server = "
                "arm_tcp_bridge.action_server:main"
            ),
            (
                "joint_feedback_node = "
                "arm_tcp_bridge.joint_feedback_node:main"
            ),
            (
                "motion_control_node = "
                "arm_tcp_bridge.motion_control_node:main"
            ),
            (
                "send_fly_queue_template = "
                "arm_tcp_bridge.fly_queue_sender_template:main"
            ),
            (
                "send_path_template = "
                "arm_tcp_bridge.path_sender_template:main"
            ),
            (
                "path_sequence_server = "
                "arm_tcp_bridge.path_sequence_action_server:main"
            ),
            (
                "send_path_sequence = "
                "arm_tcp_bridge.path_sequence_sender:main"
            ),
            (
                "offline_sequence_manager = "
                "arm_tcp_bridge.offline_sequence_manager:main"
            ),
        ],
    },
)
