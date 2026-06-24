#!/usr/bin/env python3

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GazeboInitialHoldPublisher05(Node):
    """Publish the initial zero-position hold command for the renamed active joints."""

    JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]

    def __init__(self) -> None:
        super().__init__("gazebo_initial_hold_publisher_05")

        self.done = False
        self._start_time = time.monotonic()
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Waiting to publish initial arm hold command.")

    def timer_callback(self) -> None:
        if self.done:
            return

        waited = time.monotonic() - self._start_time
        if self.publisher.get_subscription_count() == 0 and waited < 5.0:
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.JOINT_NAMES)
        point.time_from_start.sec = 1
        trajectory.points = [point]

        self.publisher.publish(trajectory)
        self.get_logger().info(
            "Published initial hold command to /arm_controller/joint_trajectory."
        )
        self.done = True


def main() -> None:
    rclpy.init()
    node = GazeboInitialHoldPublisher05()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
