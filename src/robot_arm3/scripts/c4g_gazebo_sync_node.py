#!/usr/bin/env python3

import math
import time
from enum import Enum
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SyncState(Enum):
    WAIT_INITIAL_GAZEBO = "wait_initial_gazebo"
    WAIT_C4G_FEEDBACK = "wait_c4g_feedback"
    BLEND_TO_REAL = "blend_to_real"
    LIVE_SYNC = "live_sync"


class C4GGazeboSyncNode(Node):
    """Mirror C4G joint feedback into the Gazebo arm controller."""

    def __init__(self) -> None:
        super().__init__("c4g_gazebo_sync")

        self.declare_parameter(
            "joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
        )
        self.declare_parameter(
            "c4g_joint_states_topic",
            "/c4g/joint_states",
        )
        self.declare_parameter(
            "gazebo_joint_states_topic",
            "/joint_states",
        )
        self.declare_parameter(
            "trajectory_topic",
            "/arm_controller/joint_trajectory",
        )
        self.declare_parameter("startup_delay_s", 0.5)
        self.declare_parameter("blend_duration_s", 2.0)
        self.declare_parameter("live_time_from_start_s", 0.08)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("command_publish_rate_hz", 30.0)
        self.declare_parameter(
            "signs",
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        )
        self.declare_parameter(
            "offsets",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        self._joint_names = list(
            self.get_parameter("joint_names").value
        )
        self._signs = [
            float(value)
            for value in self.get_parameter("signs").value
        ]
        self._offsets = [
            float(value)
            for value in self.get_parameter("offsets").value
        ]

        if len(self._joint_names) != 6:
            raise ValueError("joint_names must contain exactly six names")
        if len(self._signs) != 6:
            raise ValueError("signs must contain exactly six values")
        if len(self._offsets) != 6:
            raise ValueError("offsets must contain exactly six values")

        c4g_joint_states_topic = str(
            self.get_parameter("c4g_joint_states_topic").value
        )
        gazebo_joint_states_topic = str(
            self.get_parameter("gazebo_joint_states_topic").value
        )
        trajectory_topic = str(
            self.get_parameter("trajectory_topic").value
        )
        command_publish_rate_hz = float(
            self.get_parameter("command_publish_rate_hz").value
        )

        if command_publish_rate_hz <= 0.0:
            raise ValueError("command_publish_rate_hz must be positive")

        self._state = SyncState.WAIT_INITIAL_GAZEBO
        self._start_time = time.monotonic()
        self._blend_start_time: Optional[float] = None
        self._latest_c4g_positions: Optional[Dict[str, float]] = None
        self._latest_gazebo_positions: Optional[Dict[str, float]] = None
        self._latest_feedback_time: Optional[float] = None
        self._last_state_log = ""
        self._last_live_log_time = 0.0
        self._warned_missing_c4g_joints = False
        self._warned_missing_gazebo_joints = False
        self._warned_stale_feedback = False

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10,
        )
        self.create_subscription(
            JointState,
            c4g_joint_states_topic,
            self._c4g_joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            gazebo_joint_states_topic,
            self._gazebo_joint_state_callback,
            10,
        )
        self.create_timer(
            1.0 / command_publish_rate_hz,
            self._timer_callback,
        )

        self.get_logger().info(
            "C4G Gazebo sync node started. "
            f"Reading {c4g_joint_states_topic}, "
            f"commanding {trajectory_topic}."
        )

    def _c4g_joint_state_callback(self, msg: JointState) -> None:
        positions = self._extract_positions(
            msg,
            apply_mapping=True,
        )
        if positions is None:
            if not self._warned_missing_c4g_joints:
                self.get_logger().warn(
                    "Waiting for all active joints in C4G feedback."
                )
                self._warned_missing_c4g_joints = True
            return

        self._latest_c4g_positions = positions
        self._latest_feedback_time = time.monotonic()
        self._warned_missing_c4g_joints = False
        self._warned_stale_feedback = False

    def _gazebo_joint_state_callback(self, msg: JointState) -> None:
        positions = self._extract_positions(
            msg,
            apply_mapping=False,
        )
        if positions is None:
            if not self._warned_missing_gazebo_joints:
                self.get_logger().warn(
                    "Waiting for all active joints in Gazebo joint states."
                )
                self._warned_missing_gazebo_joints = True
            return

        self._latest_gazebo_positions = positions
        self._warned_missing_gazebo_joints = False

    def _timer_callback(self) -> None:
        self._log_state_once()

        if self._state == SyncState.WAIT_INITIAL_GAZEBO:
            startup_delay_s = float(
                self.get_parameter("startup_delay_s").value
            )
            if time.monotonic() - self._start_time < startup_delay_s:
                return
            if self._latest_gazebo_positions is None:
                return
            self._state = SyncState.WAIT_C4G_FEEDBACK
            return

        if self._state == SyncState.WAIT_C4G_FEEDBACK:
            if self._latest_c4g_positions is None:
                return
            if not self._feedback_is_fresh():
                return
            self._publish_blend_trajectory()
            self._blend_start_time = time.monotonic()
            self._state = SyncState.BLEND_TO_REAL
            return

        if self._state == SyncState.BLEND_TO_REAL:
            blend_duration_s = float(
                self.get_parameter("blend_duration_s").value
            )
            if (
                self._blend_start_time is not None
                and time.monotonic() - self._blend_start_time
                >= blend_duration_s
            ):
                self._state = SyncState.LIVE_SYNC
            return

        if self._state == SyncState.LIVE_SYNC:
            if not self._feedback_is_fresh():
                return
            self._publish_live_trajectory()

    def _publish_blend_trajectory(self) -> None:
        if (
            self._latest_gazebo_positions is None
            or self._latest_c4g_positions is None
        ):
            return

        blend_duration_s = float(
            self.get_parameter("blend_duration_s").value
        )

        start_positions = [
            self._latest_gazebo_positions[name]
            for name in self._joint_names
        ]
        target_positions = [
            self._latest_c4g_positions[name]
            for name in self._joint_names
        ]

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._joint_names)

        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        self._set_duration(start_point, 0.1)

        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        self._set_duration(target_point, blend_duration_s)

        trajectory.points = [start_point, target_point]
        self._trajectory_publisher.publish(trajectory)

        self.get_logger().info(
            "Published blend trajectory from Gazebo pose "
            "to latest C4G feedback: "
            + self._format_positions(target_positions)
        )

    def _publish_live_trajectory(self) -> None:
        if self._latest_c4g_positions is None:
            return

        live_time_from_start_s = float(
            self.get_parameter("live_time_from_start_s").value
        )

        point = JointTrajectoryPoint()
        point.positions = [
            self._latest_c4g_positions[name]
            for name in self._joint_names
        ]
        self._set_duration(point, live_time_from_start_s)

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._joint_names)
        trajectory.points = [point]

        self._trajectory_publisher.publish(trajectory)

        now = time.monotonic()
        if now - self._last_live_log_time >= 1.0:
            self._last_live_log_time = now
            self.get_logger().info(
                "Live sync target: "
                + self._format_positions(point.positions)
            )

    def _feedback_is_fresh(self) -> bool:
        if self._latest_feedback_time is None:
            return False

        feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value
        )
        age_s = time.monotonic() - self._latest_feedback_time

        if age_s <= feedback_timeout_s:
            return True

        if not self._warned_stale_feedback:
            self.get_logger().warn(
                "C4G feedback is stale; holding last Gazebo command."
            )
            self._warned_stale_feedback = True

        return False

    def _extract_positions(
        self,
        msg: JointState,
        apply_mapping: bool,
    ) -> Optional[Dict[str, float]]:
        positions = {}

        for index, name in enumerate(self._joint_names):
            try:
                msg_index = msg.name.index(name)
            except ValueError:
                return None

            if msg_index >= len(msg.position):
                return None

            position = float(msg.position[msg_index])
            if apply_mapping:
                position = (
                    self._signs[index] * position
                    + self._offsets[index]
                )

            if not math.isfinite(position):
                return None

            positions[name] = position

        return positions

    def _log_state_once(self) -> None:
        if self._last_state_log == self._state.value:
            return

        self._last_state_log = self._state.value
        self.get_logger().info(
            f"C4G Gazebo sync state: {self._state.value}"
        )

    @staticmethod
    def _format_positions(
        positions,
    ) -> str:
        return "[" + ", ".join(f"{value:.4f}" for value in positions) + "]"

    @staticmethod
    def _set_duration(
        point: JointTrajectoryPoint,
        seconds: float,
    ) -> None:
        whole_seconds = int(seconds)
        nanoseconds = int(
            round((seconds - whole_seconds) * 1_000_000_000)
        )

        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds


def main() -> None:
    rclpy.init()
    node = C4GGazeboSyncNode()

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
