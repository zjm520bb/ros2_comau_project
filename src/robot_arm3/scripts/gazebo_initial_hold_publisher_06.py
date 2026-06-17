#!/usr/bin/env python3

import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GazeboInitialHoldPublisher06(Node):
    """Publish the initial hold command with joint_3 at -90 degrees."""

    JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]

    def __init__(self) -> None:
        super().__init__("gazebo_initial_hold_publisher_06")

        self.done = False
        self._start_time = time.monotonic()
        self._current_positions: Optional[Dict[str, float]] = None
        self._warned_waiting_joint_states = False
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Waiting to publish initial arm hold command.")

    def joint_state_callback(self, msg: JointState) -> None:
        positions = {}
        for joint_name in self.JOINT_NAMES:
            position = self._get_joint_position(msg, joint_name)
            if position is None:
                return
            positions[joint_name] = position

        self._current_positions = positions
        self._warned_waiting_joint_states = False

    def timer_callback(self) -> None:
        if self.done:
            return

        waited = time.monotonic() - self._start_time
        if self.publisher.get_subscription_count() == 0 and waited < 5.0:
            return
        if self._current_positions is None:
            if not self._warned_waiting_joint_states:
                self.get_logger().info("Waiting for current active joint states.")
                self._warned_waiting_joint_states = True
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.JOINT_NAMES)

        start_point = JointTrajectoryPoint()
        start_point.positions = [
            self._current_positions[joint_name] for joint_name in self.JOINT_NAMES
        ]
        start_point.time_from_start.nanosec = 100_000_000

        target_point = JointTrajectoryPoint()
        target_point.positions = [
            0.0,
            0.0,
            -1.5708,
            0.0,
            0.0,
            0.0,
        ]
        target_point.time_from_start.sec = 3
        trajectory.points = [start_point, target_point]

        self.publisher.publish(trajectory)
        self.get_logger().info(
            "Published initial trajectory from current state to joint_3=-1.5708 rad."
        )
        self.done = True

    @staticmethod
    def _get_joint_position(msg: JointState, name: str) -> Optional[float]:
        try:
            index = msg.name.index(name)
        except ValueError:
            return None

        if index >= len(msg.position):
            return None

        return msg.position[index]


def main() -> None:
    rclpy.init()
    node = GazeboInitialHoldPublisher06()

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
