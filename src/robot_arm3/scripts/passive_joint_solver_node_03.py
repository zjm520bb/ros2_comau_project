#!/usr/bin/env python3

import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PassiveJointSolver03(Node):
    """Passive joint solver for the renamed joint_7 / joint_8 passive pair."""

    def __init__(self) -> None:
        super().__init__("passive_joint_solver_03")

        self.declare_parameter("joint_7_offset", 0.0)
        self.declare_parameter("joint_8_offset", 0.0)

        self._warned_missing_joints = False
        self._last_log_time = 0.0

        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_states_raw", self.joint_state_callback, 10)

        self.get_logger().info(
            "Passive joint solver 03 started. Reading /joint_states_raw and publishing /joint_states."
        )

    def joint_state_callback(self, msg: JointState) -> None:
        joint_2 = self._get_joint_position(msg, "joint_2")
        joint_3 = self._get_joint_position(msg, "joint_3")

        if joint_2 is None or joint_3 is None:
            if not self._warned_missing_joints:
                self.get_logger().warn(
                    "Waiting for joint_2 and joint_3 in /joint_states_raw."
                )
                self._warned_missing_joints = True
            return

        joint_7, joint_8 = self.solve_passive_joints(joint_2, joint_3)
        solved_msg = self._copy_joint_state(msg)
        self._set_joint_position(solved_msg, "joint_7", joint_7)
        self._set_joint_position(solved_msg, "joint_8", joint_8)
        self.joint_state_publisher.publish(solved_msg)

        now = time.monotonic()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.get_logger().info(
                "joint_2=%.6f rad, joint_3=%.6f rad -> joint_7=%.6f rad, joint_8=%.6f rad"
                % (joint_2, joint_3, joint_7, joint_8)
            )

    def solve_passive_joints(
        self, joint_2: float, joint_3: float
    ) -> Tuple[float, float]:
        joint_7_offset = float(self.get_parameter("joint_7_offset").value)
        joint_8_offset = float(self.get_parameter("joint_8_offset").value)

        joint_7 = joint_2 + joint_3 + joint_7_offset
        joint_8 = -(joint_2 + joint_3) + joint_8_offset

        return joint_7, joint_8

    @staticmethod
    def _copy_joint_state(msg: JointState) -> JointState:
        copied = JointState()
        copied.header = msg.header
        copied.name = list(msg.name)
        copied.position = list(msg.position)
        copied.velocity = list(msg.velocity)
        copied.effort = list(msg.effort)
        return copied

    @staticmethod
    def _set_joint_position(msg: JointState, name: str, position: float) -> None:
        try:
            index = msg.name.index(name)
        except ValueError:
            msg.name.append(name)
            msg.position.append(position)
            if len(msg.velocity) == len(msg.name) - 1:
                msg.velocity.append(0.0)
            if len(msg.effort) == len(msg.name) - 1:
                msg.effort.append(0.0)
            return

        if index < len(msg.position):
            msg.position[index] = position
        else:
            missing_positions = index + 1 - len(msg.position)
            msg.position.extend([0.0] * missing_positions)
            msg.position[index] = position

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
    node = PassiveJointSolver03()

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
