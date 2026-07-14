import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_tcp_bridge_interfaces.action import ExecutePath, ExecutePathSequence
from arm_tcp_bridge_interfaces.srv import SignalSequence


class PathSequenceActionServer(Node):
    def __init__(self) -> None:
        super().__init__("path_sequence_action_server")
        self.declare_parameter("action_name", "/arm/execute_path_sequence")
        self.declare_parameter("path_action_name", "/arm/execute_path")
        self.declare_parameter(
            "signal_sequence_service",
            "/arm/signal_sequence",
        )
        self.declare_parameter("joint_feedback_topic", "/c4g/joint_states")
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("default_feedback_timeout_s", 0.5)
        self.declare_parameter("path_server_wait_timeout_s", 10.0)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._feedback_lock = threading.Lock()
        self._latest_positions_deg = None
        self._latest_feedback_time = None
        self._active_path_goal = None
        self._busy = False
        self._busy_lock = threading.Lock()
        self._sequence_state_lock = threading.Lock()
        self._active_sequence_id = 0
        self._waiting_after_path = 0
        self._continue_requested = False
        self._callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_feedback_topic").value),
            self._joint_state_callback,
            20,
            callback_group=self._callback_group,
        )
        self._path_client = ActionClient(
            self,
            ExecutePath,
            str(self.get_parameter("path_action_name").value),
            callback_group=self._callback_group,
        )
        self._signal_sequence_service = self.create_service(
            SignalSequence,
            str(self.get_parameter("signal_sequence_service").value),
            self._signal_sequence,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            ExecutePathSequence,
            str(self.get_parameter("action_name").value),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            values = [
                math.degrees(message.position[message.name.index(name)])
                for name in self._joint_names
            ]
        except (ValueError, IndexError):
            return
        if not all(math.isfinite(value) for value in values):
            return
        with self._feedback_lock:
            self._latest_positions_deg = values
            self._latest_feedback_time = time.monotonic()

    def _goal_callback(self, request) -> GoalResponse:
        if not request.sequence.paths:
            return GoalResponse.REJECT
        if (
            int(request.sequence.sequence_id) <= 0
            or int(request.sequence.sequence_id) > 2000000
        ):
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        if self._active_path_goal is not None:
            self._active_path_goal.cancel_goal_async()
        with self._sequence_state_lock:
            self._continue_requested = True
        return CancelResponse.ACCEPT

    def _signal_sequence(self, request, response):
        with self._sequence_state_lock:
            if self._active_sequence_id == 0:
                response.accepted = False
                response.message = "no PATH sequence is active"
                return response
            if int(request.sequence_id) != self._active_sequence_id:
                response.accepted = False
                response.message = "sequence_id does not match active sequence"
                return response
            if self._waiting_after_path == 0:
                response.accepted = False
                response.message = (
                    "sequence is not waiting between PATH blocks"
                )
                return response
            if int(request.expected_path) not in (
                0,
                self._waiting_after_path,
            ):
                response.accepted = False
                response.message = "expected_path does not match wait point"
                return response
            self._continue_requested = True
            response.accepted = True
            response.message = "PATH sequence continue accepted"
            return response

    def _check_start(self, expected, tolerances, timeout_s):
        with self._feedback_lock:
            actual = self._latest_positions_deg
            stamp = self._latest_feedback_time
        if actual is None or stamp is None:
            return False, "no complete C4G joint feedback received"
        if time.monotonic() - stamp > timeout_s:
            return False, "C4G joint feedback is stale"
        errors = [actual[i] - float(expected[i]) for i in range(6)]
        failures = [
            (
                f"{self._joint_names[i]} expected={expected[i]:.3f}deg "
                f"actual={actual[i]:.3f}deg error={errors[i]:+.3f}deg "
                f"tolerance={tolerances[i]:.3f}deg"
            )
            for i in range(6)
            if abs(errors[i]) > tolerances[i]
        ]
        if failures:
            return (
                False,
                "C4G is not at the expected PATH start: "
                + "; ".join(failures),
            )
        return True, "start position verified"

    @staticmethod
    def _path_goal(block, generated_id: int):
        goal = ExecutePath.Goal()
        goal.path_id = int(block.path_id) or generated_id
        goal.path_type = int(block.path_type)
        goal.frames = list(block.frames)
        goal.conditions = list(block.conditions)
        goal.nodes = list(block.nodes)
        goal.start_index = int(block.start_index) or 1
        goal.end_index = int(block.end_index) or len(block.nodes)
        return goal

    async def _execute_callback(self, goal_handle):
        result = ExecutePathSequence.Result()
        completed = 0
        try:
            request = goal_handle.request
            total = len(request.sequence.paths)
            with self._sequence_state_lock:
                self._active_sequence_id = int(
                    request.sequence.sequence_id
                )
                self._waiting_after_path = 0
                self._continue_requested = False
            tolerances = [
                float(value)
                for value in request.start_tolerance_deg
            ]
            if (
                len(tolerances) != 6
                or any(value <= 0 for value in tolerances)
            ):
                raise RuntimeError(
                    "start_tolerance_deg must contain six positive values"
                )
            timeout_s = float(request.feedback_timeout_s)
            if timeout_s <= 0:
                timeout_s = float(
                    self.get_parameter("default_feedback_timeout_s").value
                )
            if not self._path_client.wait_for_server(
                timeout_sec=float(
                    self.get_parameter("path_server_wait_timeout_s").value
                )
            ):
                raise RuntimeError("C4G ExecutePath action is unavailable")

            for index, block in enumerate(request.sequence.paths):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.completed_paths = completed
                    result.message = "PATH sequence canceled"
                    return result
                if request.require_start_check:
                    valid, message = self._check_start(
                        block.expected_start_deg, tolerances, timeout_s
                    )
                    if not valid:
                        raise RuntimeError(message)

                feedback = ExecutePathSequence.Feedback()
                feedback.state = "sending"
                feedback.current_path = index + 1
                feedback.total_paths = total
                goal_handle.publish_feedback(feedback)

                path_goal = self._path_goal(
                    block,
                    int(request.sequence.sequence_id) * 1000 + index + 1,
                )
                send_future = self._path_client.send_goal_async(
                    path_goal,
                    feedback_callback=lambda msg, path_index=index: (
                        self._forward_feedback(
                            goal_handle, path_index, total, msg.feedback
                        )
                    ),
                )
                self._active_path_goal = await send_future
                if not self._active_path_goal.accepted:
                    raise RuntimeError(
                        f"C4G rejected PATH {index + 1}: {block.name}"
                    )
                wrapped = await self._active_path_goal.get_result_async()
                self._active_path_goal = None
                if (
                    wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or not wrapped.result.success
                ):
                    raise RuntimeError(
                        f"PATH {index + 1} failed: {wrapped.result.message}"
                    )
                completed += 1
                if bool(block.wait_after) and index + 1 < total:
                    with self._sequence_state_lock:
                        self._waiting_after_path = index + 1
                        self._continue_requested = False
                    feedback = ExecutePathSequence.Feedback()
                    feedback.state = "waiting_between_paths"
                    feedback.current_path = index + 1
                    feedback.total_paths = total
                    goal_handle.publish_feedback(feedback)
                    while True:
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            result.success = False
                            result.completed_paths = completed
                            result.message = (
                                "PATH sequence canceled while waiting"
                            )
                            return result
                        with self._sequence_state_lock:
                            if self._continue_requested:
                                self._waiting_after_path = 0
                                self._continue_requested = False
                                break
                        time.sleep(0.05)
                    feedback = ExecutePathSequence.Feedback()
                    feedback.state = "resumed_between_paths"
                    feedback.current_path = index + 1
                    feedback.total_paths = total
                    goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True
            result.completed_paths = completed
            result.message = "PATH sequence execution finished"
            return result
        except Exception as exc:
            self.get_logger().error(f"PATH sequence failed: {exc}")
            goal_handle.abort()
            result.success = False
            result.completed_paths = completed
            result.message = str(exc)
            return result
        finally:
            self._active_path_goal = None
            with self._sequence_state_lock:
                self._active_sequence_id = 0
                self._waiting_after_path = 0
                self._continue_requested = False
            with self._busy_lock:
                self._busy = False

    @staticmethod
    def _forward_feedback(goal_handle, path_index, total, path_feedback):
        if not goal_handle.is_active:
            return
        feedback = ExecutePathSequence.Feedback()
        feedback.state = path_feedback.state
        feedback.current_path = path_index + 1
        feedback.total_paths = total
        feedback.current_node = path_feedback.current_node
        goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        self._server.destroy()
        self._path_client.destroy()
        self.destroy_service(self._signal_sequence_service)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathSequenceActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
