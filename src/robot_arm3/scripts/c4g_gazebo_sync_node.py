#!/usr/bin/env python3

import math
import time
from enum import Enum
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_tcp_bridge_interfaces.srv import ResetJoints
from std_srvs.srv import SetBool, Trigger


class SyncState(Enum):
    DISABLED = "disabled"
    ERROR = "error"
    WAIT_INITIAL_GAZEBO = "wait_initial_gazebo"
    WAIT_C4G_FEEDBACK = "wait_c4g_feedback"
    PREPARE_TELEPORT = "prepare_teleport"
    RESET_TO_REAL = "reset_to_real"
    VERIFY_RESET = "verify_reset"
    RESUME_PASSIVE_SOLVER = "resume_passive_solver"
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
            "passive_joint_names",
            ["joint_7", "joint_8"],
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
        self.declare_parameter(
            "initial_gazebo_positions",
            [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0],
        )
        self.declare_parameter("initial_pose_tolerance_rad", 0.01)
        self.declare_parameter("initial_pose_stable_duration_s", 0.5)
        self.declare_parameter("initial_pose_timeout_s", 15.0)
        self.declare_parameter("blend_duration_s", 2.0)
        self.declare_parameter("initial_sync_mode", "blend")
        self.declare_parameter(
            "mirror_joint_reset_service",
            "/gazebo/reset_robot_joints_for_mirror",
        )
        self.declare_parameter(
            "passive_command_topic",
            "/internal_passive_controller/commands",
        )
        self.declare_parameter(
            "passive_solver_enable_service",
            "/gazebo_passive_joint_controller_06/set_enabled",
        )
        self.declare_parameter("require_passive_solver_handover", True)
        self.declare_parameter(
            "active_joint_lower_limits",
            [-3.14, -1.3, -4.0317, -47.12, -2.18, -47.12],
        )
        self.declare_parameter(
            "active_joint_upper_limits",
            [3.14, 1.3, 0.0, 47.12, 2.18, 47.12],
        )
        self.declare_parameter(
            "passive_joint_lower_limits",
            [-3.14, -3.14],
        )
        self.declare_parameter(
            "passive_joint_upper_limits",
            [3.14, 3.14],
        )
        self.declare_parameter("teleport_limit_tolerance_rad", 0.001)
        self.declare_parameter("joint_reset_timeout_s", 20.0)
        self.declare_parameter("reset_tolerance_rad", 0.01)
        self.declare_parameter("reset_velocity_tolerance_rad_s", 0.02)
        self.declare_parameter("reset_stable_duration_s", 0.2)
        self.declare_parameter("coupling_offset", 1.5708)
        self.declare_parameter("enabled_on_start", True)
        self.declare_parameter("require_initial_gazebo_pose", True)
        self.declare_parameter("live_time_from_start_s", 0.08)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("command_publish_rate_hz", 30.0)
        self.declare_parameter("live_log_change_threshold_rad", 0.001)
        self.declare_parameter("live_log_heartbeat_s", 20.0)
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
        self._passive_joint_names = list(
            self.get_parameter("passive_joint_names").value
        )
        self._gazebo_joint_names = (
            self._joint_names + self._passive_joint_names
        )
        self._signs = [
            float(value)
            for value in self.get_parameter("signs").value
        ]
        self._offsets = [
            float(value)
            for value in self.get_parameter("offsets").value
        ]
        self._initial_gazebo_positions = [
            float(value)
            for value in self.get_parameter(
                "initial_gazebo_positions"
            ).value
        ]
        self._active_joint_lower_limits = [
            float(value)
            for value in self.get_parameter(
                "active_joint_lower_limits"
            ).value
        ]
        self._active_joint_upper_limits = [
            float(value)
            for value in self.get_parameter(
                "active_joint_upper_limits"
            ).value
        ]
        self._passive_joint_lower_limits = [
            float(value)
            for value in self.get_parameter(
                "passive_joint_lower_limits"
            ).value
        ]
        self._passive_joint_upper_limits = [
            float(value)
            for value in self.get_parameter(
                "passive_joint_upper_limits"
            ).value
        ]

        if len(self._joint_names) != 6:
            raise ValueError("joint_names must contain exactly six names")
        if len(self._passive_joint_names) != 2:
            raise ValueError(
                "passive_joint_names must contain exactly two names"
            )
        if len(set(self._gazebo_joint_names)) != len(
            self._gazebo_joint_names
        ):
            raise ValueError("Gazebo joint names must be unique")
        if len(self._signs) != 6:
            raise ValueError("signs must contain exactly six values")
        if len(self._offsets) != 6:
            raise ValueError("offsets must contain exactly six values")
        if len(self._initial_gazebo_positions) != len(self._joint_names):
            raise ValueError(
                "initial_gazebo_positions must contain one value per joint"
            )
        if not all(
            math.isfinite(value)
            for value in self._initial_gazebo_positions
        ):
            raise ValueError(
                "initial_gazebo_positions must contain finite values"
            )
        for label, lower, upper, expected_size in (
            (
                "active joint limits",
                self._active_joint_lower_limits,
                self._active_joint_upper_limits,
                len(self._joint_names),
            ),
            (
                "passive joint limits",
                self._passive_joint_lower_limits,
                self._passive_joint_upper_limits,
                len(self._passive_joint_names),
            ),
        ):
            if len(lower) != expected_size or len(upper) != expected_size:
                raise ValueError(
                    f"{label} must contain {expected_size} values"
                )
            if any(
                not math.isfinite(low)
                or not math.isfinite(high)
                or low >= high
                for low, high in zip(lower, upper)
            ):
                raise ValueError(
                    f"{label} must be finite lower/upper pairs"
                )

        initial_pose_tolerance_rad = float(
            self.get_parameter("initial_pose_tolerance_rad").value
        )
        initial_pose_stable_duration_s = float(
            self.get_parameter("initial_pose_stable_duration_s").value
        )
        initial_pose_timeout_s = float(
            self.get_parameter("initial_pose_timeout_s").value
        )
        if initial_pose_tolerance_rad <= 0.0:
            raise ValueError("initial_pose_tolerance_rad must be positive")
        if initial_pose_stable_duration_s < 0.0:
            raise ValueError(
                "initial_pose_stable_duration_s must be non-negative"
            )
        if initial_pose_timeout_s < 0.0:
            raise ValueError("initial_pose_timeout_s must be non-negative")

        reset_velocity_tolerance_rad_s = float(
            self.get_parameter(
                "reset_velocity_tolerance_rad_s"
            ).value
        )
        reset_stable_duration_s = float(
            self.get_parameter("reset_stable_duration_s").value
        )
        if reset_velocity_tolerance_rad_s < 0.0:
            raise ValueError(
                "reset_velocity_tolerance_rad_s must be non-negative"
            )
        if reset_stable_duration_s < 0.0:
            raise ValueError(
                "reset_stable_duration_s must be non-negative"
            )
        teleport_limit_tolerance_rad = float(
            self.get_parameter("teleport_limit_tolerance_rad").value
        )
        if teleport_limit_tolerance_rad < 0.0:
            raise ValueError(
                "teleport_limit_tolerance_rad must be non-negative"
            )

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
        live_log_change_threshold_rad = float(
            self.get_parameter("live_log_change_threshold_rad").value
        )
        live_log_heartbeat_s = float(
            self.get_parameter("live_log_heartbeat_s").value
        )
        if live_log_change_threshold_rad < 0.0:
            raise ValueError(
                "live_log_change_threshold_rad must be non-negative"
            )
        if live_log_heartbeat_s <= 0.0:
            raise ValueError("live_log_heartbeat_s must be positive")

        self._state = (
            self._initial_sync_wait_state()
            if bool(self.get_parameter("enabled_on_start").value)
            else SyncState.DISABLED
        )
        self._start_time = time.monotonic()
        self._initial_pose_reached_time: Optional[float] = None
        self._initial_pose_timeout_logged = False
        self._blend_start_time: Optional[float] = None
        self._reset_start_time: Optional[float] = None
        self._reset_stable_start_time: Optional[float] = None
        self._reset_future = None
        self._passive_solver_future = None
        self._passive_solver_paused = False
        self._reset_target_positions: Optional[Dict[str, float]] = None
        self._latest_c4g_positions: Optional[Dict[str, float]] = None
        self._latest_gazebo_positions: Optional[Dict[str, float]] = None
        self._latest_gazebo_velocities: Optional[Dict[str, float]] = None
        self._latest_feedback_time: Optional[float] = None
        self._last_state_log = ""
        self._last_live_log_time = 0.0
        self._last_logged_live_positions = None
        self._warned_missing_c4g_joints = False
        self._warned_missing_gazebo_joints = False
        self._warned_stale_feedback = False

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10,
        )
        self._joint_reset_client = self.create_client(
            ResetJoints,
            str(self.get_parameter("mirror_joint_reset_service").value),
        )
        self._passive_command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("passive_command_topic").value),
            10,
        )
        self._passive_solver_client = self.create_client(
            SetBool,
            str(self.get_parameter("passive_solver_enable_service").value),
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
        self.create_service(Trigger, "~/enable", self._enable_sync)

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
        state = self._extract_gazebo_state(msg)
        if state is None:
            if not self._warned_missing_gazebo_joints:
                self.get_logger().warn(
                    "Waiting for positions and velocities of all active and "
                    "passive joints in Gazebo joint states."
                )
                self._warned_missing_gazebo_joints = True
            return

        positions, velocities = state
        self._latest_gazebo_positions = positions
        self._latest_gazebo_velocities = velocities
        self._warned_missing_gazebo_joints = False

    def _timer_callback(self) -> None:
        self._log_state_once()

        if self._state == SyncState.DISABLED:
            return

        if self._state == SyncState.ERROR:
            return

        if self._state == SyncState.WAIT_INITIAL_GAZEBO:
            startup_delay_s = float(
                self.get_parameter("startup_delay_s").value
            )
            now = time.monotonic()
            if now - self._start_time < startup_delay_s:
                return
            if self._latest_gazebo_positions is None:
                return

            tolerance = float(
                self.get_parameter("initial_pose_tolerance_rad").value
            )
            errors = [
                abs(
                    self._latest_gazebo_positions[name]
                    - self._initial_gazebo_positions[index]
                )
                for index, name in enumerate(self._joint_names)
            ]
            if max(errors) > tolerance:
                self._initial_pose_reached_time = None
                self._log_initial_pose_timeout(now, errors)
                return

            if self._initial_pose_reached_time is None:
                self._initial_pose_reached_time = now
                self.get_logger().info(
                    "Gazebo reached the initial pose; waiting for it to "
                    "remain stable before C4G synchronization"
                )
                return

            stable_duration_s = float(
                self.get_parameter(
                    "initial_pose_stable_duration_s"
                ).value
            )
            if now - self._initial_pose_reached_time < stable_duration_s:
                return

            self.get_logger().info(
                "Gazebo initial pose verified; C4G synchronization is now "
                "allowed"
            )
            self._state = SyncState.WAIT_C4G_FEEDBACK
            return

        if self._state == SyncState.WAIT_C4G_FEEDBACK:
            if self._latest_c4g_positions is None:
                return
            if not self._feedback_is_fresh():
                return
            initial_sync_mode = str(
                self.get_parameter("initial_sync_mode").value
            ).strip().lower()
            if initial_sync_mode == "blend":
                self._publish_blend_trajectory()
                self._blend_start_time = time.monotonic()
                self._state = SyncState.BLEND_TO_REAL
            elif initial_sync_mode == "teleport":
                if not self._prepare_teleport():
                    return
                self._state = SyncState.PREPARE_TELEPORT
            else:
                raise ValueError(
                    "initial_sync_mode must be 'teleport' or 'blend'"
                )
            return

        if self._state == SyncState.PREPARE_TELEPORT:
            self._poll_teleport_prepare()
            return

        if self._state == SyncState.RESET_TO_REAL:
            self._poll_joint_reset()
            return

        if self._state == SyncState.VERIFY_RESET:
            now = time.monotonic()
            if self._reset_is_stable():
                if self._reset_stable_start_time is None:
                    self._reset_stable_start_time = now
                    return
                stable_duration_s = float(
                    self.get_parameter("reset_stable_duration_s").value
                )
                if (
                    now - self._reset_stable_start_time
                    < stable_duration_s
                ):
                    return
                self.get_logger().info(
                    "Gazebo eight-joint reset verified and stable; entering "
                    "live C4G mirror"
                )
                if not bool(
                    self.get_parameter(
                        "require_passive_solver_handover"
                    ).value
                ):
                    self._state = SyncState.LIVE_SYNC
                    self._publish_live_trajectory()
                    return
                if not self._set_passive_solver_enabled(True):
                    self._state = SyncState.ERROR
                    return
                self._state = SyncState.RESUME_PASSIVE_SOLVER
                return
            self._reset_stable_start_time = None
            timeout_s = float(
                self.get_parameter("joint_reset_timeout_s").value
            )
            if (
                self._reset_start_time is not None
                and time.monotonic() - self._reset_start_time > timeout_s
            ):
                self.get_logger().error(
                    "Gazebo did not reach a stable eight-joint reset pose "
                    f"within {timeout_s:.2f}s; automatic retry is disabled"
                )
                self._abort_teleport()
            return

        if self._state == SyncState.RESUME_PASSIVE_SOLVER:
            self._poll_passive_solver_resume()
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

    def _enable_sync(self, _request, response):
        if self._state not in (SyncState.DISABLED, SyncState.ERROR):
            response.success = True
            response.message = f"sync is already {self._state.value}"
            return response
        self._start_time = time.monotonic()
        self._initial_pose_reached_time = None
        self._initial_pose_timeout_logged = False
        self._reset_start_time = None
        self._reset_stable_start_time = None
        self._reset_future = None
        self._passive_solver_future = None
        self._reset_target_positions = None
        self._last_live_log_time = 0.0
        self._last_logged_live_positions = None
        self._state = self._initial_sync_wait_state()
        response.success = True
        response.message = (
            "C4G Gazebo synchronization enabled; "
            f"next state is {self._state.value}"
        )
        return response

    def _initial_sync_wait_state(self) -> SyncState:
        if bool(
            self.get_parameter("require_initial_gazebo_pose").value
        ):
            return SyncState.WAIT_INITIAL_GAZEBO
        return SyncState.WAIT_C4G_FEEDBACK

    def _prepare_teleport(self) -> bool:
        if self._latest_c4g_positions is None:
            return False

        active = {
            name: self._latest_c4g_positions[name]
            for name in self._joint_names
        }
        joint_2 = active["joint_2"]
        joint_3 = active["joint_3"]
        coupling_offset = float(
            self.get_parameter("coupling_offset").value
        )
        passive_1 = joint_2 + joint_3 + coupling_offset
        passive_2 = -passive_1
        self._reset_target_positions = dict(active)
        self._reset_target_positions[self._passive_joint_names[0]] = (
            passive_1
        )
        self._reset_target_positions[self._passive_joint_names[1]] = (
            passive_2
        )

        limit_error = self._teleport_limit_error()
        if limit_error is not None:
            self.get_logger().error(
                "Refusing C4G teleport because its target is outside "
                f"the Gazebo joint limits: {limit_error}"
            )
            self._state = SyncState.ERROR
            return False

        if not bool(
            self.get_parameter("require_passive_solver_handover").value
        ):
            return True

        if not self._passive_solver_client.service_is_ready():
            self._passive_solver_client.wait_for_service(timeout_sec=0.0)
            self.get_logger().error(
                "Cannot start C4G teleport: passive solver enable service "
                "is unavailable"
            )
            self._state = SyncState.ERROR
            return False

        request = SetBool.Request()
        request.data = False
        self._passive_solver_future = (
            self._passive_solver_client.call_async(request)
        )
        self.get_logger().info(
            "Paused passive joint solver before C4G teleport"
        )
        return True

    def _poll_teleport_prepare(self) -> None:
        if self._passive_solver_future is None:
            if self._start_joint_reset():
                self._state = SyncState.RESET_TO_REAL
            return
        if not self._passive_solver_future.done():
            return

        response = self._passive_solver_future.result()
        self._passive_solver_future = None
        if response is None or not response.success:
            message = "service returned no response"
            if response is not None:
                message = response.message
            self.get_logger().error(
                "Cannot start C4G teleport: failed to pause passive "
                f"joint solver: {message}"
            )
            self._state = SyncState.ERROR
            return

        self._passive_solver_paused = True
        if self._start_joint_reset():
            self._state = SyncState.RESET_TO_REAL

    def _start_joint_reset(self) -> bool:
        if self._reset_target_positions is None:
            return False
        if not self._joint_reset_client.service_is_ready():
            self._joint_reset_client.wait_for_service(timeout_sec=0.0)
            return False

        request = ResetJoints.Request()
        request.joint_names = list(self._gazebo_joint_names)
        request.positions = [
            self._reset_target_positions[name]
            for name in self._gazebo_joint_names
        ]
        self._reset_future = self._joint_reset_client.call_async(request)
        self._reset_start_time = time.monotonic()
        self._reset_stable_start_time = None
        self.get_logger().info(
            "Requested no-pause C4G mirror reset to feedback: "
            + self._format_positions(
                [
                    self._reset_target_positions[name]
                    for name in self._joint_names
                ]
            )
        )
        return True

    def _teleport_limit_error(self) -> Optional[str]:
        if self._reset_target_positions is None:
            return "no reset target is available"
        tolerance = float(
            self.get_parameter("teleport_limit_tolerance_rad").value
        )
        limits = zip(
            self._gazebo_joint_names,
            self._active_joint_lower_limits
            + self._passive_joint_lower_limits,
            self._active_joint_upper_limits
            + self._passive_joint_upper_limits,
        )
        for name, lower, upper in limits:
            value = self._reset_target_positions[name]
            if value < lower - tolerance or value > upper + tolerance:
                return (
                    f"{name}={value:.6f} rad is outside "
                    f"[{lower:.6f}, {upper:.6f}] rad"
                )
        return None

    def _poll_joint_reset(self) -> None:
        if self._reset_future is None:
            self._state = SyncState.WAIT_C4G_FEEDBACK
            return
        if not self._reset_future.done():
            timeout_s = float(
                self.get_parameter("joint_reset_timeout_s").value
            )
            if (
                self._reset_start_time is not None
                and time.monotonic() - self._reset_start_time > timeout_s
            ):
                self.get_logger().error(
                    "Timed out waiting for Gazebo joint reset service; "
                    "automatic retry is disabled"
                )
                self._reset_future = None
                self._abort_teleport()
            return

        response = self._reset_future.result()
        self._reset_future = None
        if response is None or not response.success:
            message = "service returned no response"
            if response is not None:
                message = response.message
            self.get_logger().error(f"Gazebo joint reset failed: {message}")
            self._abort_teleport()
            return

        self._publish_teleport_handover()
        self._reset_stable_start_time = None
        self._state = SyncState.VERIFY_RESET

    def _abort_teleport(self) -> None:
        if self._passive_solver_paused:
            self._set_passive_solver_enabled(True)
            self._passive_solver_paused = False
        self._state = SyncState.ERROR

    def _publish_teleport_handover(self) -> None:
        if self._reset_target_positions is None:
            return
        command = Float64MultiArray()
        command.data = [
            self._reset_target_positions[name]
            for name in self._passive_joint_names
        ]
        self._passive_command_publisher.publish(command)
        self._publish_live_trajectory()
        self.get_logger().info(
            "Primed arm and passive controller targets for the C4G "
            "teleport handover"
        )

    def _set_passive_solver_enabled(self, enabled: bool) -> bool:
        if not bool(
            self.get_parameter("require_passive_solver_handover").value
        ):
            return True
        if not self._passive_solver_client.service_is_ready():
            self._passive_solver_client.wait_for_service(timeout_sec=0.0)
            self.get_logger().error(
                "Passive solver enable service is unavailable during "
                "C4G teleport handover"
            )
            return False
        request = SetBool.Request()
        request.data = enabled
        self._passive_solver_future = (
            self._passive_solver_client.call_async(request)
        )
        return True

    def _poll_passive_solver_resume(self) -> None:
        if self._passive_solver_future is None:
            self._state = SyncState.ERROR
            return
        if not self._passive_solver_future.done():
            return
        response = self._passive_solver_future.result()
        self._passive_solver_future = None
        if response is None or not response.success:
            message = "service returned no response"
            if response is not None:
                message = response.message
            self.get_logger().error(
                "Failed to resume passive solver after C4G teleport: "
                f"{message}"
            )
            self._abort_teleport()
            return
        self._passive_solver_paused = False
        self.get_logger().info(
            "C4G teleport handover verified; entering live C4G mirror"
        )
        self._state = SyncState.LIVE_SYNC
        self._publish_live_trajectory()

    def _reset_is_stable(self) -> bool:
        if (
            self._reset_target_positions is None
            or self._latest_gazebo_positions is None
            or self._latest_gazebo_velocities is None
        ):
            return False
        tolerance = float(
            self.get_parameter("reset_tolerance_rad").value
        )
        velocity_tolerance = float(
            self.get_parameter(
                "reset_velocity_tolerance_rad_s"
            ).value
        )
        for name in self._gazebo_joint_names:
            if (
                abs(
                    self._latest_gazebo_positions[name]
                    - self._reset_target_positions[name]
                )
                > tolerance
            ):
                return False
            if (
                abs(self._latest_gazebo_velocities[name])
                > velocity_tolerance
            ):
                return False
        return True

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
        change_threshold = float(
            self.get_parameter("live_log_change_threshold_rad").value
        )
        heartbeat_s = float(
            self.get_parameter("live_log_heartbeat_s").value
        )
        target_changed = (
            self._last_logged_live_positions is None
            or any(
                abs(current - previous) >= change_threshold
                for current, previous in zip(
                    point.positions,
                    self._last_logged_live_positions,
                )
            )
        )
        heartbeat_due = now - self._last_live_log_time >= heartbeat_s
        if target_changed or heartbeat_due:
            self._last_live_log_time = now
            self._last_logged_live_positions = list(point.positions)
            label = "target changed" if target_changed else "heartbeat"
            self.get_logger().info(
                f"Live sync {label}: "
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

    def _extract_gazebo_state(
        self,
        msg: JointState,
    ) -> Optional[tuple[Dict[str, float], Dict[str, float]]]:
        positions = {}
        velocities = {}

        for name in self._gazebo_joint_names:
            try:
                msg_index = msg.name.index(name)
            except ValueError:
                return None

            if (
                msg_index >= len(msg.position)
                or msg_index >= len(msg.velocity)
            ):
                return None

            position = float(msg.position[msg_index])
            velocity = float(msg.velocity[msg_index])
            if not math.isfinite(position) or not math.isfinite(velocity):
                return None

            positions[name] = position
            velocities[name] = velocity

        return positions, velocities

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

    def _log_initial_pose_timeout(
        self,
        now: float,
        errors,
    ) -> None:
        timeout_s = float(
            self.get_parameter("initial_pose_timeout_s").value
        )
        if (
            timeout_s <= 0.0
            or now - self._start_time <= timeout_s
            or self._initial_pose_timeout_logged
        ):
            return

        worst_index = max(range(len(errors)), key=errors.__getitem__)
        worst_joint = self._joint_names[worst_index]
        self.get_logger().error(
            "Gazebo has not reached the initial pose within "
            f"{timeout_s:.2f}s; still waiting and C4G synchronization "
            f"remains blocked. Largest error: {worst_joint}="
            f"{errors[worst_index]:.6f} rad"
        )
        self._initial_pose_timeout_logged = True

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
