#!/usr/bin/env python3

import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GazeboPassiveJointController03(Node):
    """Command renamed Gazebo passive joints from the renamed active joint states."""

    def __init__(self) -> None:
        super().__init__("gazebo_passive_joint_controller_03")

        self._active_joint_positions: Optional[Tuple[float, float]] = None
        self._warned_missing_joints = False
        self._last_log_time = 0.0

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/internal_passive_controller/commands",
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_timer(1.0 / 50.0, self.timer_callback)

        self.get_logger().info(
            "Gazebo passive joint controller 03 started. "
            "Reading joint_2 and joint_3, commanding joint_7 and joint_8."
        )

    def joint_state_callback(self, msg: JointState) -> None:
        joint_2 = self._get_joint_position(msg, "joint_2")
        joint_3 = self._get_joint_position(msg, "joint_3")

        if joint_2 is None or joint_3 is None:
            if not self._warned_missing_joints:
                self.get_logger().warn("Waiting for joint_2 and joint_3 in /joint_states.")
                self._warned_missing_joints = True
            return

        self._active_joint_positions = (joint_2, joint_3)
        self._warned_missing_joints = False

    def timer_callback(self) -> None:
        if self._active_joint_positions is None:
            return

        joint_2, joint_3 = self._active_joint_positions
        joint_7 = joint_2 + joint_3
        joint_8 = -(joint_2 + joint_3)

        command = Float64MultiArray()
        command.data = [joint_7, joint_8]
        self.command_publisher.publish(command)

        now = time.monotonic()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.get_logger().info(
                "commanding joint_7=%.6f rad, joint_8=%.6f rad from joint_2=%.6f rad, joint_3=%.6f rad"
                % (joint_7, joint_8, joint_2, joint_3)
            )

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
    node = GazeboPassiveJointController03()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
