import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from arm_tcp_bridge.path_sequence_io import save_sequence
from arm_tcp_bridge_interfaces.action import ExecutePathSequence
from arm_tcp_bridge_interfaces.msg import PathBlock, PathSequence
from arm_tcp_bridge_interfaces.srv import (
    GetOfflineSequenceStatus,
    ResetJoints,
)


class OfflineSequenceManager(Node):
    """Collect reviewed PATH blocks and export or execute a sequence."""

    def __init__(self) -> None:
        super().__init__("offline_path_sequence_manager")
        self.declare_parameter("prepared_path_topic", "/offline/prepared_path")
        self.declare_parameter(
            "motion_active_topic",
            "/offline/motion_active",
        )
        self.declare_parameter(
            "sequence_action_name",
            "/arm/execute_path_sequence",
        )
        self.declare_parameter(
            "preview_sequence_action_name",
            "/sim/arm/execute_path_sequence",
        )
        self.declare_parameter(
            "joint_reset_service",
            "/gazebo/reset_robot_joints",
        )
        self.declare_parameter(
            "mirror_enable_service",
            "/c4g_gazebo_sync/enable",
        )
        self.declare_parameter("export_path", "/tmp/prepared_path.c4gseq.yaml")
        self.declare_parameter("sequence_id", 1)
        self.declare_parameter("sequence_name", "offline_sequence")
        self.declare_parameter("robot_model", "robot_arm3")
        self.declare_parameter("coupling_offset", 1.5708)
        self.declare_parameter("start_tolerance_deg", 0.5)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("merge_compatible_paths", True)
        self.declare_parameter("merge_tolerance_deg", 0.5)
        self.declare_parameter("max_path_nodes", 1000)

        self._lock = threading.Lock()
        self._recording = False
        self._motion_active = False
        self._draft = None
        self._paths = []
        self._active_goal = None
        self._active_preview_goal = None
        self.create_subscription(
            PathBlock,
            str(self.get_parameter("prepared_path_topic").value),
            self._draft_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("motion_active_topic").value),
            self._motion_active_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._sequence_client = ActionClient(
            self,
            ExecutePathSequence,
            str(self.get_parameter("sequence_action_name").value),
        )
        self._preview_client = ActionClient(
            self,
            ExecutePathSequence,
            str(self.get_parameter("preview_sequence_action_name").value),
        )
        self._reset_client = self.create_client(
            ResetJoints,
            str(self.get_parameter("joint_reset_service").value),
        )
        self._mirror_enable_client = self.create_client(
            Trigger,
            str(self.get_parameter("mirror_enable_service").value),
        )
        self.create_service(Trigger, "~/accept_draft", self._accept)
        self.create_service(Trigger, "~/reject_draft", self._reject)
        self.create_service(
            Trigger,
            "~/start_recording",
            self._start_recording,
        )
        self.create_service(
            Trigger,
            "~/stop_recording",
            self._stop_recording,
        )
        self.create_service(
            GetOfflineSequenceStatus,
            "~/get_status",
            self._get_status,
        )
        self.create_service(
            SetBool,
            "~/set_draft_end_wait",
            self._set_draft_end_wait,
        )
        self.create_service(
            SetBool,
            "~/set_draft_wait_after",
            self._set_draft_wait_after,
        )
        self.create_service(Trigger, "~/undo_last", self._undo)
        self.create_service(Trigger, "~/clear", self._clear)
        self.create_service(Trigger, "~/export", self._export)
        self.create_service(Trigger, "~/preview_all", self._preview_all)
        self.create_service(Trigger, "~/send", self._send)

    def _draft_callback(self, block: PathBlock) -> None:
        with self._lock:
            if not self._recording:
                self.get_logger().info(
                    f"Ignored PATH {block.name!r}: recording is stopped"
                )
                return
            if self._draft is not None:
                self.get_logger().error(
                    f"Ignored PATH {block.name!r}: review the existing "
                    "draft before planning another motion"
                )
                return
            self._draft = block
        self.get_logger().info(
            f"Draft PATH ready: {block.name}; nodes={len(block.nodes)}. "
            "Review the Gazebo preview, then call ~/accept_draft or "
            "~/reject_draft."
        )

    def _motion_active_callback(self, message: Bool) -> None:
        with self._lock:
            self._motion_active = bool(message.data)

    def _start_recording(self, _request, response):
        with self._lock:
            if self._recording:
                response.success = False
                response.message = "PATH recording is already active"
                return response
            if self._motion_active:
                response.success = False
                response.message = (
                    "cannot start recording while a simulated motion is active"
                )
                return response
            if self._draft is not None:
                response.success = False
                response.message = "reject the stale draft before recording"
                return response
            if (
                self._active_goal is not None
                or self._active_preview_goal is not None
            ):
                response.success = False
                response.message = "a PATH sequence is active"
                return response
            self._recording = True
        response.success = True
        response.message = (
            "PATH recording started; subsequent successful motions "
            "will produce drafts"
        )
        return response

    def _stop_recording(self, _request, response):
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = "PATH recording is not active"
                return response
            reason = self._cannot_finalize_reason()
            if reason:
                response.success = False
                response.message = reason
                return response
            self._recording = False
        response.success = True
        response.message = "PATH recording stopped; sequence is ready"
        return response

    def _cannot_finalize_reason(self) -> str:
        if self._motion_active:
            return "wait for the active simulated motion to finish"
        if self._draft is not None:
            return "accept or reject the current draft first"
        for block in self._paths:
            if block.nodes and block.nodes[-1].wait:
                return (
                    f"PATH {block.name!r} ends with a node wait; add a "
                    "compatible following segment or remove that wait"
                )
        return ""

    def _get_status(self, _request, response):
        with self._lock:
            reason = self._cannot_finalize_reason()
            sequence_ready = (
                not self._recording
                and not reason
                and bool(self._paths)
                and self._active_goal is None
                and self._active_preview_goal is None
            )
            response.recording = self._recording
            response.draft_pending = self._draft is not None
            response.accepted_paths = len(self._paths)
            response.motion_active = self._motion_active
            response.preview_active = self._active_preview_goal is not None
            response.send_active = self._active_goal is not None
            response.can_stop_recording = self._recording and not reason
            response.can_preview = sequence_ready
            response.can_export = sequence_ready
            response.can_send = sequence_ready
            mode = "recording" if self._recording else "stopped"
            response.message = (
                f"{mode}; accepted_paths={len(self._paths)}; "
                f"draft_pending={self._draft is not None}"
            )
            if reason:
                response.message += f"; blocked={reason}"
        return response

    def _set_draft_end_wait(self, request, response):
        with self._lock:
            if not self._recording or self._draft is None:
                response.success = False
                response.message = "no recording draft is available"
                return response
            if not self._draft.nodes:
                response.success = False
                response.message = "draft PATH contains no nodes"
                return response
            node = self._draft.nodes[-1]
            if bool(request.data) and node.motion_type == node.SEG_VIA:
                response.success = False
                response.message = "SEG_VIA cannot be a wait node"
                return response
            if bool(request.data) and self._draft.wait_after:
                response.success = False
                response.message = (
                    "disable draft wait_after before enabling node wait"
                )
                return response
            node.wait = bool(request.data)
            if node.wait:
                node.fly = False
            response.success = True
            response.message = (
                f"draft endpoint wait set to {node.wait}"
            )
            return response

    def _set_draft_wait_after(self, request, response):
        with self._lock:
            if not self._recording or self._draft is None:
                response.success = False
                response.message = "no recording draft is available"
                return response
            if (
                bool(request.data)
                and self._draft.nodes
                and self._draft.nodes[-1].wait
            ):
                response.success = False
                response.message = (
                    "disable the draft endpoint node wait before "
                    "enabling wait_after"
                )
                return response
            self._draft.wait_after = bool(request.data)
            response.success = True
            response.message = (
                f"draft PATH wait_after set to {self._draft.wait_after}"
            )
            return response

    def _sequence(self) -> PathSequence:
        sequence = PathSequence()
        sequence.sequence_id = int(self.get_parameter("sequence_id").value)
        sequence.name = str(self.get_parameter("sequence_name").value)
        sequence.robot_model = str(self.get_parameter("robot_model").value)
        sequence.coupling_offset = float(
            self.get_parameter("coupling_offset").value
        )
        sequence.stop_on_error = True
        sequence.paths = list(self._paths)
        return sequence

    def _accept(self, _request, response):
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = "start PATH recording before accepting"
                return response
            if self._draft is None:
                response.success = False
                response.message = "no draft PATH is waiting for review"
                return response
            draft = self._draft
            merged = False
            if (
                bool(self.get_parameter("merge_compatible_paths").value)
                and self._paths
                and self._compatible(self._paths[-1], draft)
            ):
                previous = self._paths[-1]
                previous.nodes.extend(draft.nodes)
                previous.end_index = len(previous.nodes)
                previous.expected_end_deg = draft.expected_end_deg
                previous.wait_after = draft.wait_after
                merged = True
            else:
                if (
                    self._paths
                    and self._paths[-1].nodes
                    and self._paths[-1].nodes[-1].wait
                ):
                    response.success = False
                    response.message = (
                        "previous PATH ends with a node wait and this draft "
                        "cannot merge with it; use wait_after for a "
                        "different PATH type"
                    )
                    return response
                self._paths.append(draft)
            self._draft = None
            count = len(self._paths)
        response.success = True
        action = (
            "merged with previous compatible PATH"
            if merged
            else "accepted"
        )
        response.message = (
            f"draft {action}; sequence now contains {count} PATH block(s)"
        )
        return response

    def _compatible(self, left: PathBlock, right: PathBlock) -> bool:
        if left.wait_after:
            return False
        if int(left.path_type) != int(right.path_type):
            return False
        if left.frames != right.frames or left.conditions != right.conditions:
            return False
        tolerance = float(
            self.get_parameter("merge_tolerance_deg").value
        )
        if any(
            abs(float(left.expected_end_deg[index])
                - float(right.expected_start_deg[index])) > tolerance
            for index in range(6)
        ):
            return False
        return (
            len(left.nodes) + len(right.nodes)
            <= int(self.get_parameter("max_path_nodes").value)
        )

    def _reject(self, _request, response):
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = "PATH recording is not active"
                return response
            draft = self._draft
            self._draft = None
        if draft is None:
            response.success = False
            response.message = "no draft PATH exists"
            return response
        reset_started = self._reset_to_expected(draft.expected_start_deg)
        response.success = True
        response.message = (
            "draft rejected; Gazebo reset to the segment start requested"
            if reset_started
            else "draft rejected; Gazebo reset service is unavailable"
        )
        return response

    def _undo(self, _request, response):
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = "undo is available while recording"
                return response
            if self._motion_active or self._draft is not None:
                response.success = False
                response.message = (
                    "finish the active motion and review its draft first"
                )
                return response
            if not self._paths:
                response.success = False
                response.message = "sequence is empty"
                return response
            removed = self._paths.pop()
        reset_started = self._reset_to_expected(removed.expected_start_deg)
        response.success = True
        suffix = (
            "; Gazebo reset to its start requested"
            if reset_started
            else "; Gazebo reset service is unavailable"
        )
        response.message = f"removed PATH {removed.name!r}{suffix}"
        return response

    def _clear(self, _request, response):
        with self._lock:
            if (
                self._motion_active
                or self._active_goal is not None
                or self._active_preview_goal is not None
            ):
                response.success = False
                response.message = "cannot clear while motion is active"
                return response
            self._paths.clear()
            self._draft = None
            self._recording = False
        response.success = True
        response.message = (
            "offline PATH sequence cleared; recording is stopped"
        )
        return response

    def _export(self, _request, response):
        with self._lock:
            if self._recording:
                response.success = False
                response.message = "stop PATH recording before export"
                return response
            if self._draft is not None:
                response.success = False
                response.message = (
                    "accept or reject the current draft before export"
                )
                return response
            sequence = self._sequence()
        if not sequence.paths:
            response.success = False
            response.message = "cannot export an empty sequence"
            return response
        path = str(self.get_parameter("export_path").value)
        try:
            save_sequence(path, sequence)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f"exported {len(sequence.paths)} PATH block(s) to {path}"
        )
        return response

    def _send(self, _request, response):
        with self._lock:
            if self._recording:
                response.success = False
                response.message = "stop PATH recording before sending"
                return response
            if self._draft is not None:
                response.success = False
                response.message = (
                    "accept or reject the current draft before sending"
                )
                return response
            sequence = self._sequence()
        if not sequence.paths:
            response.success = False
            response.message = "cannot send an empty sequence"
            return response
        if self._active_goal is not None:
            response.success = False
            response.message = "a sequence is already executing"
            return response
        if not self._sequence_client.wait_for_server(timeout_sec=0.0):
            response.success = False
            response.message = "PATH sequence action server is unavailable"
            return response
        goal = ExecutePathSequence.Goal()
        goal.sequence = sequence
        goal.require_start_check = True
        tolerance = float(self.get_parameter("start_tolerance_deg").value)
        goal.start_tolerance_deg = [tolerance] * 6
        goal.feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value
        )
        if self._mirror_enable_client.service_is_ready():
            self._mirror_enable_client.call_async(Trigger.Request())
        future = self._sequence_client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self.get_logger().info(
                f"PATH sequence {msg.feedback.state}: "
                f"{msg.feedback.current_path}/{msg.feedback.total_paths}"
            ),
        )
        future.add_done_callback(self._goal_sent)
        response.success = True
        response.message = (
            "PATH sequence submitted for start validation and execution"
        )
        return response

    def _preview_all(self, _request, response):
        with self._lock:
            if self._recording:
                response.success = False
                response.message = (
                    "stop PATH recording before final preview"
                )
                return response
            if self._draft is not None:
                response.success = False
                response.message = (
                    "accept or reject the current draft before final preview"
                )
                return response
            sequence = self._sequence()
        if not sequence.paths:
            response.success = False
            response.message = "cannot preview an empty sequence"
            return response
        if self._active_preview_goal is not None:
            response.success = False
            response.message = "a final sequence preview is already active"
            return response
        if not self._reset_client.service_is_ready():
            response.success = False
            response.message = "Gazebo joint reset service is unavailable"
            return response
        request = self._reset_request(
            sequence.paths[0].expected_start_deg
        )
        future = self._reset_client.call_async(request)
        future.add_done_callback(
            lambda reset_future: self._start_final_preview(
                reset_future, sequence
            )
        )
        response.success = True
        response.message = (
            "Gazebo reset requested; final sequence preview will start "
            "after reset"
        )
        return response

    def _reset_request(self, expected_start_deg):
        radians = [
            math.radians(value)
            for value in expected_start_deg
        ]
        coupling_offset = float(
            self.get_parameter("coupling_offset").value
        )
        joint_7 = radians[1] + radians[2] + coupling_offset
        request = ResetJoints.Request()
        request.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4",
            "joint_5", "joint_6", "joint_7", "joint_8",
        ]
        request.positions = radians + [joint_7, -joint_7]
        return request

    def _reset_to_expected(self, expected_start_deg) -> bool:
        if not self._reset_client.service_is_ready():
            return False
        future = self._reset_client.call_async(
            self._reset_request(expected_start_deg)
        )
        future.add_done_callback(self._log_edit_reset)
        return True

    def _log_edit_reset(self, future):
        result = future.result()
        if result is None or not result.success:
            message = "no response" if result is None else result.message
            self.get_logger().error(f"Gazebo edit reset failed: {message}")
            return
        self.get_logger().info("Gazebo restored to the rejected segment start")

    def _start_final_preview(self, reset_future, sequence):
        reset = reset_future.result()
        if reset is None or not reset.success:
            message = "no reset response" if reset is None else reset.message
            self.get_logger().error(f"Cannot start final preview: {message}")
            return
        if not self._preview_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error(
                "Simulation PATH sequence action is unavailable"
            )
            return
        goal = ExecutePathSequence.Goal()
        goal.sequence = sequence
        goal.require_start_check = False
        goal.start_tolerance_deg = [0.5] * 6
        goal.feedback_timeout_s = 0.5
        future = self._preview_client.send_goal_async(goal)
        future.add_done_callback(self._preview_goal_sent)

    def _preview_goal_sent(self, future):
        self._active_preview_goal = future.result()
        if (
            self._active_preview_goal is None
            or not self._active_preview_goal.accepted
        ):
            self.get_logger().error("Final Gazebo PATH preview was rejected")
            self._active_preview_goal = None
            return
        result = self._active_preview_goal.get_result_async()
        result.add_done_callback(self._preview_finished)

    def _preview_finished(self, future):
        self.get_logger().info(
            "Final Gazebo preview: " + future.result().result.message
        )
        self._active_preview_goal = None

    def _goal_sent(self, future):
        self._active_goal = future.result()
        if self._active_goal is None or not self._active_goal.accepted:
            self.get_logger().error("PATH sequence goal was rejected")
            self._active_goal = None
            return
        result_future = self._active_goal.get_result_async()
        result_future.add_done_callback(self._sequence_finished)

    def _sequence_finished(self, future):
        wrapped = future.result()
        self.get_logger().info(wrapped.result.message)
        self._active_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = OfflineSequenceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
