
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
    maintainer="Jiaming Zhang",
    maintainer_email="you@example.com",
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
        ],
    },
)
