
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
from arm_tcp_bridge_interfaces.action import ExecuteCommand


FINAL_OK = "Movement finished"
FINAL_UNAVAILABLE = "Movement unavailable"
FINAL_ERROR = "ERROR"


class ArmExecuteActionServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_execute_action_server")

        self.declare_parameter(
            "robot_ip",
            "130.149.138.38",
        )
        self.declare_parameter("cmd_port", 8000)

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
        tcp_debug = bool(
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

        self._busy_lock = threading.Lock()
        self._busy = False

        self._action_server = ActionServer(
            self,
            ExecuteCommand,
            "arm/execute",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            "Arm Execute Action Server started. "
            f"Command target: {robot_ip}:{cmd_port}"
        )

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

        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    "Rejecting goal because server is busy"
                )
                return GoalResponse.REJECT

            self._busy = True

        return GoalResponse.ACCEPT

    def cancel_callback(
        self,
        goal_handle,
    ) -> CancelResponse:
        del goal_handle

        self.get_logger().warn(
            "Cancel rejected: robot-side HOLD/STOP "
            "has not been implemented"
        )

        return CancelResponse.REJECT

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

        strict_echo = bool(
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

    def destroy_node(self) -> None:
        try:
            self._cmd_client.close()
        finally:
            self._action_server.destroy()
            super().destroy_node()


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


