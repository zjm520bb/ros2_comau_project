
import threading
import time

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_tcp_bridge.tcp_client import TcpClient
from arm_tcp_bridge.path_protocol import (
    PathValidationError,
    build_upload_commands,
    execute_path_command,
    validate_path_goal,
)
from arm_tcp_bridge_interfaces.action import (
    ExecuteCommand,
    ExecutePath,
)
from arm_tcp_bridge_interfaces.msg import PathEvent


FINAL_OK = "Movement finished"
FINAL_UNAVAILABLE = "Movement unavailable"
FINAL_ERROR = "ERROR"
FINAL_ABORTED = "Motion aborted"
FINAL_JOINTS_PREFIX = "JOINTS:"
FINAL_POSE_PREFIX = "POSE:"
PATH_WAITING_PREFIX = "PATH_WAITING:"


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
    return bool(value)


class ArmExecuteActionServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_execute_action_server")

        self.declare_parameter(
            "robot_ip",
            "130.149.138.38",
        )
        self.declare_parameter("cmd_port", 8000)
        self.declare_parameter("control_port", 8002)
        self.declare_parameter(
            "enable_motion_control",
            False,
        )
        self.declare_parameter(
            "path_action_name",
            "arm/execute_path",
        )
        self.declare_parameter(
            "enable_path_protocol",
            False,
        )
        self.declare_parameter(
            "c4g_protocol_version",
            1,
        )
        self.declare_parameter("max_path_nodes", 1000)
        self.declare_parameter(
            "path_upload_timeout_s",
            10.0,
        )
        self.declare_parameter(
            "path_execution_timeout_s",
            3600.0,
        )

        self.declare_parameter("connect_attempts", 10)
        self.declare_parameter(
            "connect_backoff_s",
            0.5,
        )

        self.declare_parameter(
            "connect_timeout_s",
            3.0,
        )
        self.declare_parameter(
            "io_timeout_s",
            0.5,
        )
        self.declare_parameter(
            "handshake_timeout_s",
            3.0,
        )
        self.declare_parameter(
            "final_timeout_s",
            120.0,
        )

        self.declare_parameter(
            "strict_echo_check",
            True,
        )
        self.declare_parameter(
            "max_command_bytes",
            254,
        )
        self.declare_parameter(
            "tcp_debug",
            False,
        )

        robot_ip = str(
            self.get_parameter("robot_ip").value
        )
        cmd_port = int(
            self.get_parameter("cmd_port").value
        )
        connect_timeout_s = float(
            self.get_parameter(
                "connect_timeout_s"
            ).value
        )
        io_timeout_s = float(
            self.get_parameter(
                "io_timeout_s"
            ).value
        )
        max_command_bytes = int(
            self.get_parameter(
                "max_command_bytes"
            ).value
        )
        tcp_debug = _as_bool(
            self.get_parameter("tcp_debug").value
        )

        self._cmd_client = TcpClient(
            host=robot_ip,
            port=cmd_port,
            connect_timeout_s=connect_timeout_s,
            io_timeout_s=io_timeout_s,
            max_send_bytes=max_command_bytes,
            debug=tcp_debug,
        )

        self._control_enabled = _as_bool(
            self.get_parameter(
                "enable_motion_control"
            ).value
        )
        control_port = int(
            self.get_parameter("control_port").value
        )
        self._control_client = TcpClient(
            host=robot_ip,
            port=control_port,
            connect_timeout_s=connect_timeout_s,
            io_timeout_s=io_timeout_s,
            max_send_bytes=max_command_bytes,
            debug=tcp_debug,
        )

        self._busy_lock = threading.Lock()
        self._busy = False
        self._path_enabled = _as_bool(
            self.get_parameter(
                "enable_path_protocol"
            ).value
        ) and int(
            self.get_parameter(
                "c4g_protocol_version"
            ).value
        ) >= 2

        self._action_server = ActionServer(
            self,
            ExecuteCommand,
            "arm/execute",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self._path_action_server = ActionServer(
            self,
            ExecutePath,
            str(
                self.get_parameter(
                    "path_action_name"
                ).value
            ),
            execute_callback=self.execute_path_callback,
            goal_callback=self.path_goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self._path_event_publisher = self.create_publisher(
            PathEvent,
            "arm/path_events",
            20,
        )

        self.get_logger().info(
            "Arm Execute Action Server started. "
            f"Command target: {robot_ip}:{cmd_port}"
        )
        if self._control_enabled:
            self.get_logger().info(
                "Motion control target enabled: "
                f"{robot_ip}:{control_port}"
            )
        self.get_logger().info(
            "C4G PATH protocol "
            + ("enabled" if self._path_enabled else "disabled")
        )

    def _reserve_server(self) -> bool:
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def goal_callback(
        self,
        goal_request: ExecuteCommand.Goal,
    ) -> GoalResponse:
        command = goal_request.command.strip()

        if not command:
            self.get_logger().warn(
                "Rejecting empty command"
            )
            return GoalResponse.REJECT

        try:
            encoded = command.encode(
                "ascii",
                errors="strict",
            )
        except UnicodeEncodeError:
            self.get_logger().warn(
                "Rejecting command containing "
                "non-ASCII characters"
            )
            return GoalResponse.REJECT

        max_command_bytes = int(
            self.get_parameter(
                "max_command_bytes"
            ).value
        )

        if len(encoded) > max_command_bytes:
            self.get_logger().warn(
                "Rejecting command because it is too long: "
                f"{len(encoded)} > {max_command_bytes}"
            )
            return GoalResponse.REJECT

        if not self._reserve_server():
            self.get_logger().warn(
                "Rejecting goal because server is busy"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def path_goal_callback(
        self,
        goal_request: ExecutePath.Goal,
    ) -> GoalResponse:
        if not self._path_enabled:
            self.get_logger().warn(
                "Rejecting PATH: enable_path_protocol is false "
                "or c4g_protocol_version is below 2"
            )
            return GoalResponse.REJECT
        try:
            path = validate_path_goal(
                goal_request,
                int(
                    self.get_parameter(
                        "max_path_nodes"
                    ).value
                ),
            )
            if any(node.wait for node in path.nodes) and not self._control_enabled:
                raise PathValidationError(
                    "PATH contains wait nodes but motion control is disabled"
                )
            commands = build_upload_commands(path)
            commands.append(execute_path_command(path))
            max_bytes = int(
                self.get_parameter(
                    "max_command_bytes"
                ).value
            )
            for command in commands:
                size = len(command.encode("ascii"))
                if size > max_bytes:
                    raise PathValidationError(
                        f"generated command is {size} bytes; "
                        f"maximum is {max_bytes}"
                    )
        except (PathValidationError, ValueError) as exc:
            self.get_logger().warn(
                f"Rejecting invalid PATH: {exc}"
            )
            return GoalResponse.REJECT

        if not self._reserve_server():
            self.get_logger().warn(
                "Rejecting PATH because server is busy"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(
        self,
        goal_handle,
    ) -> CancelResponse:
        del goal_handle

        if not self._control_enabled:
            self.get_logger().warn(
                "Cancel rejected: motion control port is disabled"
            )
            return CancelResponse.REJECT

        try:
            self._send_control_command("abortMotion")

        except Exception as exc:
            self.get_logger().warn(
                f"Cancel rejected: abortMotion failed: {exc}"
            )
            return CancelResponse.REJECT

        self.get_logger().warn(
            "Cancel accepted: abortMotion sent to C4G"
        )
        return CancelResponse.ACCEPT

    def _set_not_busy(self) -> None:
        with self._busy_lock:
            self._busy = False

    def _ensure_cmd_connection(self) -> None:
        attempts = int(
            self.get_parameter(
                "connect_attempts"
            ).value
        )
        backoff_s = float(
            self.get_parameter(
                "connect_backoff_s"
            ).value
        )

        self._cmd_client.ensure_connected(
            max_attempts=attempts,
            backoff_s=backoff_s,
        )

    def execute_callback(
        self,
        goal_handle,
    ) -> ExecuteCommand.Result:
        feedback = ExecuteCommand.Feedback()
        result = ExecuteCommand.Result()

        command = goal_handle.request.command.strip()

        strict_echo = _as_bool(
            self.get_parameter(
                "strict_echo_check"
            ).value
        )
        handshake_timeout_s = float(
            self.get_parameter(
                "handshake_timeout_s"
            ).value
        )
        final_timeout_s = float(
            self.get_parameter(
                "final_timeout_s"
            ).value
        )

        try:
            self._publish_feedback(
                goal_handle,
                feedback,
                "ensuring_cmd_connection",
            )

            self._ensure_cmd_connection()

            self._publish_feedback(
                goal_handle,
                feedback,
                f"sending_command: {command}",
            )

            self._cmd_client.send_msg(command)

            self._publish_feedback(
                goal_handle,
                feedback,
                "waiting_echo",
            )

            echo = self._recv_with_deadline(
                client=self._cmd_client,
                timeout_s=handshake_timeout_s,
            )

            if strict_echo and echo != command:
                self._cmd_client.close()

                goal_handle.abort()
                result.success = False
                result.message = (
                    "Echo mismatch: "
                    f"expected {command!r}, got {echo!r}"
                )
                return result

            self._publish_feedback(
                goal_handle,
                feedback,
                "sending_start",
            )

            self._cmd_client.send_msg("start")

            self._publish_feedback(
                goal_handle,
                feedback,
                "waiting_final",
            )

            final_message = self._recv_with_deadline(
                client=self._cmd_client,
                timeout_s=final_timeout_s,
            )

            if final_message == FINAL_OK:
                goal_handle.succeed()
                result.success = True
                result.message = FINAL_OK
                return result

            if final_message == FINAL_UNAVAILABLE:
                goal_handle.abort()
                result.success = False
                result.message = FINAL_UNAVAILABLE
                return result

            if final_message == FINAL_ERROR:
                self._cmd_client.close()

                goal_handle.abort()
                result.success = False
                result.message = FINAL_ERROR
                return result

            if final_message == FINAL_ABORTED:
                result.success = False
                result.message = FINAL_ABORTED
                if self._is_cancel_requested(goal_handle):
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            if (
                final_message.startswith(
                    FINAL_JOINTS_PREFIX
                )
                or final_message.startswith(
                    FINAL_POSE_PREFIX
                )
            ):
                goal_handle.succeed()
                result.success = True
                result.message = final_message
                return result

            self._cmd_client.close()

            goal_handle.abort()
            result.success = False
            result.message = (
                "Unknown final response: "
                f"{final_message!r}"
            )
            return result

        except Exception as exc:
            self._cmd_client.close()

            self.get_logger().error(
                f"Command execution failed: {exc}"
            )

            goal_handle.abort()
            result.success = False
            result.message = f"Exception: {exc}"
            return result

        finally:
            self._set_not_busy()

    def execute_path_callback(
        self,
        goal_handle,
    ) -> ExecutePath.Result:
        result = ExecutePath.Result()
        path_id = int(goal_handle.request.path_id)
        upload_complete = False

        try:
            path = validate_path_goal(
                goal_handle.request,
                int(
                    self.get_parameter(
                        "max_path_nodes"
                    ).value
                ),
            )
            commands = build_upload_commands(path)
            uploaded_nodes = 0

            self._publish_path_feedback(
                goal_handle,
                "uploading",
                uploaded_nodes,
                0,
                False,
            )
            for command in commands:
                if self._is_cancel_requested(goal_handle):
                    self._best_effort_abort_path(path_id)
                    goal_handle.canceled()
                    result.success = False
                    result.message = "PATH upload canceled"
                    result.executed_nodes = 0
                    return result

                final = self._perform_command(
                    command,
                    float(
                        self.get_parameter(
                            "path_upload_timeout_s"
                        ).value
                    ),
                )
                if final != FINAL_OK:
                    raise RuntimeError(
                        f"{command.split(':', 1)[0]} failed: {final}"
                    )
                if command.startswith("commitPathNode:"):
                    uploaded_nodes += 1
                    self._publish_path_feedback(
                        goal_handle,
                        "uploading",
                        uploaded_nodes,
                        0,
                        False,
                    )
            upload_complete = True

            self._publish_path_feedback(
                goal_handle,
                "executing",
                uploaded_nodes,
                path.start_index,
                False,
            )

            def on_intermediate(message: str) -> None:
                self._handle_path_intermediate(
                    goal_handle,
                    uploaded_nodes,
                    message,
                )

            final = self._perform_command(
                execute_path_command(path),
                float(
                    self.get_parameter(
                        "path_execution_timeout_s"
                    ).value
                ),
                on_intermediate=on_intermediate,
            )

            if final == FINAL_OK:
                goal_handle.succeed()
                result.success = True
                result.message = "PATH execution finished"
                result.executed_nodes = (
                    abs(path.end_index - path.start_index) + 1
                )
                return result

            if final == FINAL_ABORTED:
                result.success = False
                result.message = FINAL_ABORTED
                result.executed_nodes = 0
                if self._is_cancel_requested(goal_handle):
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            raise RuntimeError(f"PATH execution failed: {final}")

        except Exception as exc:
            self._cmd_client.close()
            if not upload_complete:
                self._best_effort_abort_path(path_id)
            elif self._control_enabled:
                try:
                    self._send_control_command("abortMotion")
                except Exception:
                    self._control_client.close()
            self.get_logger().error(
                f"PATH execution failed: {exc}"
            )
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.executed_nodes = 0
            return result
        finally:
            self._set_not_busy()

    def _handle_path_intermediate(
        self,
        goal_handle,
        uploaded_nodes: int,
        message: str,
    ) -> None:
        if message.startswith(PATH_WAITING_PREFIX):
            fields = message[len(PATH_WAITING_PREFIX):].split(",")
            node_index = int(fields[1]) if len(fields) > 1 else 0
            self._publish_path_feedback(
                goal_handle,
                "waiting",
                uploaded_nodes,
                node_index,
                True,
            )
            event = PathEvent()
            event.path_id = int(fields[0])
            event.node_index = node_index
            event.event_type = PathEvent.WAITING
            event.description = "segment_wait"
            self._path_event_publisher.publish(event)
            return

        if message.startswith("PATH_EVENT:"):
            fields = message[len("PATH_EVENT:"):].split(",")
            if len(fields) < 3:
                return
            handler = int(fields[2])
            event = PathEvent()
            event.path_id = int(fields[0])
            event.node_index = int(fields[1])
            event.condition_handler = handler
            if handler == 10:
                event.event_type = PathEvent.NODE_START
            elif handler == 11:
                event.event_type = PathEvent.NODE_END
            elif handler == 12:
                event.event_type = PathEvent.VIA
            else:
                event.event_type = PathEvent.CONDITION
            event.description = "condition_handler"
            self._path_event_publisher.publish(event)

    def _perform_command(
        self,
        command: str,
        timeout_s: float,
        on_intermediate=None,
    ) -> str:
        self._ensure_cmd_connection()
        self._cmd_client.send_msg(command)
        echo = self._recv_with_deadline(
            client=self._cmd_client,
            timeout_s=float(
                self.get_parameter(
                    "handshake_timeout_s"
                ).value
            ),
        )
        if _as_bool(
            self.get_parameter(
                "strict_echo_check"
            ).value
        ) and echo != command:
            raise RuntimeError(
                f"Echo mismatch: expected {command!r}, got {echo!r}"
            )
        self._cmd_client.send_msg("start")

        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"Timeout waiting for {command.split(':', 1)[0]}"
                )
            message = self._recv_with_deadline(
                client=self._cmd_client,
                timeout_s=remaining,
            )
            if message in (
                FINAL_OK,
                FINAL_UNAVAILABLE,
                FINAL_ERROR,
                FINAL_ABORTED,
            ):
                return message
            if on_intermediate is None:
                return message
            on_intermediate(message)

    def _best_effort_abort_path(self, path_id: int) -> None:
        if not self._path_enabled:
            return
        try:
            self._perform_command(
                f"abortPathUpload:{path_id}",
                float(
                    self.get_parameter(
                        "path_upload_timeout_s"
                    ).value
                ),
            )
        except Exception:
            self._cmd_client.close()

    @staticmethod
    def _publish_path_feedback(
        goal_handle,
        state: str,
        uploaded_nodes: int,
        current_node: int,
        waiting: bool,
    ) -> None:
        feedback = ExecutePath.Feedback()
        feedback.state = state
        feedback.uploaded_nodes = uploaded_nodes
        feedback.current_node = current_node
        feedback.waiting = waiting
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _publish_feedback(
        goal_handle,
        feedback: ExecuteCommand.Feedback,
        state: str,
    ) -> None:
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _recv_with_deadline(
        client: TcpClient,
        timeout_s: float,
    ) -> str:
        deadline = time.monotonic() + timeout_s

        while True:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Timeout waiting for robot response"
                )

            try:
                return client.recv_msg()

            except RuntimeError as exc:
                if "timeout" in str(exc).lower():
                    continue

                raise

    @staticmethod
    def _is_cancel_requested(goal_handle) -> bool:
        cancel_state = getattr(
            goal_handle,
            "is_cancel_requested",
            False,
        )
        if callable(cancel_state):
            return bool(cancel_state())
        return bool(cancel_state)

    def destroy_node(self) -> None:
        try:
            self._cmd_client.close()
            self._control_client.close()
        finally:
            self._action_server.destroy()
            self._path_action_server.destroy()
            super().destroy_node()

    def _send_control_command(
        self,
        command: str,
    ) -> str:
        attempts = int(
            self.get_parameter(
                "connect_attempts"
            ).value
        )
        backoff_s = float(
            self.get_parameter(
                "connect_backoff_s"
            ).value
        )

        self._control_client.ensure_connected(
            max_attempts=attempts,
            backoff_s=backoff_s,
        )
        self._control_client.send_msg(command)
        return self._recv_with_deadline(
            client=self._control_client,
            timeout_s=float(
                self.get_parameter(
                    "handshake_timeout_s"
                ).value
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ArmExecuteActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
