import os
from glob import glob

from setuptools import find_packages, setup


package_name = "robot_arm3_sensors"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jiaming",
    maintainer_email="jm867019644@gmail.com",
    description=(
        "Sensor abstraction and simulated sensor data streams "
        "for the robot_arm3 cell"
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "sensor_sim_node = "
                "robot_arm3_sensors.sensor_sim_node:main"
            ),
        ],
    },
)
