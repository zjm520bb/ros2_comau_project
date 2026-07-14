import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from arm_tcp_bridge.tcp_client import TcpClient
from arm_tcp_bridge_interfaces.srv import (
    GetPathState,
    SignalPath,
)

PATH_STATE_PREFIX = "PATH_STATE:"


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
    return bool(value)


def _path_state_fields(message: str) -> list[str]:
    return [
        field.strip()
        for field in message[len(PATH_STATE_PREFIX):].split(",")
    ]


class MotionControlNode(Node):
    def __init__(self) -> None:
        super().__init__("comau_motion_control")

        self.declare_parameter(
            "robot_ip",
            "130.149.138.38",
        )
        self.declare_parameter("control_port", 8002)
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
            "response_timeout_s",
            3.0,
        )
        self.declare_parameter(
            "max_command_bytes",
            254,
        )
        self.declare_parameter(
            "tcp_debug",
            False,
        )
        self.declare_parameter(
            "service_prefix",
            "arm",
        )

        robot_ip = str(
            self.get_parameter("robot_ip").value
        )
        control_port = int(
            self.get_parameter("control_port").value
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

        self._client = TcpClient(
            host=robot_ip,
            port=control_port,
            connect_timeout_s=connect_timeout_s,
            io_timeout_s=io_timeout_s,
            max_send_bytes=max_command_bytes,
            debug=tcp_debug,
        )

        prefix = str(
            self.get_parameter("service_prefix").value
        ).strip("/")

        self._pause_service = self.create_service(
            Trigger,
            f"{prefix}/pause_motion",
            lambda request, response: self._handle_trigger(
                request,
                response,
                "pauseMotion",
            ),
        )
        self._resume_service = self.create_service(
            Trigger,
            f"{prefix}/resume_motion",
            lambda request, response: self._handle_trigger(
                request,
                response,
                "resumeMotion",
            ),
        )
        self._abort_service = self.create_service(
            Trigger,
            f"{prefix}/abort_motion",
            lambda request, response: self._handle_trigger(
                request,
                response,
                "abortMotion",
            ),
        )
        self._signal_path_service = self.create_service(
            SignalPath,
            f"{prefix}/signal_path",
            self._handle_signal_path,
        )
        self._path_state_service = self.create_service(
            GetPathState,
            f"{prefix}/get_path_state",
            self._handle_path_state,
        )

        self.get_logger().info(
            "Motion control node started. "
            f"Control target: {robot_ip}:{control_port}; "
            f"services: {prefix}/pause_motion, "
            f"{prefix}/resume_motion, {prefix}/abort_motion, "
            f"{prefix}/signal_path, {prefix}/get_path_state"
        )

    def _handle_signal_path(
        self,
        request: SignalPath.Request,
        response: SignalPath.Response,
    ) -> SignalPath.Response:
        command = (
            f"continuePath:{int(request.path_id)},"
            f"{int(request.expected_node)}"
        )
        try:
            message = self._send_control_command(command)
        except Exception as exc:
            self._client.close()
            response.accepted = False
            response.message = str(exc)
            return response
        response.accepted = message.startswith("PATH_CONTINUED:")
        response.message = message
        return response

    def _handle_path_state(
        self,
        request: GetPathState.Request,
        response: GetPathState.Response,
    ) -> GetPathState.Response:
        command = f"getPathState:{int(request.path_id)}"
        try:
            message = self._send_control_command(command)
        except Exception as exc:
            self._client.close()
            response.known = False
            response.state = "ERROR"
            response.current_node = 0
            response.waiting = False
            response.message = str(exc)
            return response

        response.message = message
        if not message.startswith(PATH_STATE_PREFIX):
            response.known = False
            response.state = "UNKNOWN"
            response.current_node = 0
            response.waiting = False
            return response

        fields = _path_state_fields(message)
        if len(fields) < 4:
            response.known = False
            response.state = "INVALID"
            response.current_node = 0
            response.waiting = False
            return response

        response.known = int(fields[0]) == int(request.path_id)
        response.state = fields[1]
        response.current_node = int(fields[2])
        response.waiting = fields[3] == "1"
        return response

    def _handle_trigger(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        command: str,
    ) -> Trigger.Response:
        del request

        try:
            message = self._send_control_command(command)
        except Exception as exc:
            self._client.close()
            response.success = False
            response.message = f"{command} failed: {exc}"
            return response

        response.success = not message.startswith("Unknown")
        response.message = message
        return response

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

        self._client.ensure_connected(
            max_attempts=attempts,
            backoff_s=backoff_s,
        )
        self._client.send_msg(command)
        return self._recv_with_deadline(
            timeout_s=float(
                self.get_parameter(
                    "response_timeout_s"
                ).value
            ),
        )

    def _recv_with_deadline(
        self,
        timeout_s: float,
    ) -> str:
        deadline = time.monotonic() + timeout_s

        while True:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Timeout waiting for control response"
                )

            try:
                return self._client.recv_msg()

            except RuntimeError as exc:
                if "timeout" in str(exc).lower():
                    continue

                raise

    def destroy_node(self) -> None:
        self._client.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MotionControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
