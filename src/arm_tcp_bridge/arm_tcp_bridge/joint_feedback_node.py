import math
import threading
from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_tcp_bridge.tcp_client import TcpClient


MESSAGE_PREFIX = "JOINTS:"


class JointFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("comau_joint_feedback")

        self.declare_parameter(
            "robot_ip",
            "130.149.138.38",
        )
        self.declare_parameter(
            "feedback_port",
            8001,
        )
        self.declare_parameter(
            "connect_timeout_s",
            3.0,
        )
        self.declare_parameter(
            "io_timeout_s",
            1.0,
        )
        self.declare_parameter(
            "reconnect_backoff_s",
            1.0,
        )
        self.declare_parameter(
            "tcp_debug",
            False,
        )

        self.declare_parameter(
            "joint_state_topic",
            "/joint_states",
        )

        # C4G axis 1..6 correspond directly to Gazebo joint_1..joint_6.
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

        robot_ip = str(
            self.get_parameter("robot_ip").value
        )
        feedback_port = int(
            self.get_parameter(
                "feedback_port"
            ).value
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
        tcp_debug = bool(
            self.get_parameter("tcp_debug").value
        )
        joint_state_topic = str(
            self.get_parameter(
                "joint_state_topic"
            ).value
        )

        self._joint_names = list(
            self.get_parameter(
                "joint_names"
            ).value
        )

        if len(self._joint_names) != 6:
            raise ValueError(
                "Parameter joint_names must contain "
                "exactly six joint names"
            )

        self._client = TcpClient(
            host=robot_ip,
            port=feedback_port,
            connect_timeout_s=connect_timeout_s,
            io_timeout_s=io_timeout_s,
            max_message_bytes=1024,
            debug=tcp_debug,
        )

        self._publisher = self.create_publisher(
            JointState,
            joint_state_topic,
            10,
        )

        self._stop_event = threading.Event()

        self._worker = threading.Thread(
            target=self._feedback_loop,
            name="comau_joint_feedback_worker",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            "Joint feedback node started. "
            f"Feedback target: {robot_ip}:{feedback_port}; "
            f"publishing: {joint_state_topic}; "
            f"joint names: {self._joint_names}"
        )

    def _feedback_loop(self) -> None:
        reconnect_backoff_s = float(
            self.get_parameter(
                "reconnect_backoff_s"
            ).value
        )

        while (
            rclpy.ok()
            and not self._stop_event.is_set()
        ):
            try:
                if not self._client.is_connected():
                    self.get_logger().info(
                        "Connecting to C4G feedback port..."
                    )

                    self._client.ensure_connected(
                        max_attempts=1,
                        backoff_s=reconnect_backoff_s,
                    )

                    self.get_logger().info(
                        "Connected to C4G feedback port"
                    )

                message = self._client.recv_msg()

                positions_deg = self._parse_joint_message(
                    message
                )

                self._publish_joint_state(
                    positions_deg
                )

            except RuntimeError as exc:
                if "timeout" in str(exc).lower():
                    continue

                self.get_logger().warn(
                    f"Feedback TCP error: {exc}"
                )

                self._client.close()

                self._stop_event.wait(
                    reconnect_backoff_s
                )

            except ValueError as exc:
                self.get_logger().warn(
                    f"Invalid feedback message: {exc}"
                )

            except Exception as exc:
                self.get_logger().error(
                    f"Unexpected feedback error: {exc}"
                )

                self._client.close()

                self._stop_event.wait(
                    reconnect_backoff_s
                )

    @staticmethod
    def _parse_joint_message(
        message: str,
    ) -> list[float]:
        if not message.startswith(MESSAGE_PREFIX):
            raise ValueError(
                f"Unexpected message type: {message!r}"
            )

        values_text = message[
            len(MESSAGE_PREFIX):
        ]

        fields = values_text.split(",")

        if len(fields) != 6:
            raise ValueError(
                "Expected six joint values, "
                f"received {len(fields)}: {message!r}"
            )

        try:
            values = [
                float(field.strip())
                for field in fields
            ]

        except ValueError as exc:
            raise ValueError(
                "Feedback contains a non-numeric "
                f"joint value: {message!r}"
            ) from exc

        if not all(
            math.isfinite(value)
            for value in values
        ):
            raise ValueError(
                f"Feedback contains NaN or infinity: "
                f"{message!r}"
            )

        return values

    def _publish_joint_state(
        self,
        positions_deg: Sequence[float],
    ) -> None:
        message = JointState()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Direct mapping:
        # C4G axis 1 -> joint_1
        # C4G axis 2 -> joint_2
        # ...
        # C4G axis 6 -> joint_6
        message.name = self._joint_names

        # C4G returns degrees; ROS uses radians.
        message.position = [
            math.radians(value)
            for value in positions_deg
        ]

        self._publisher.publish(message)

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._client.close()

        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = JointFeedbackNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
