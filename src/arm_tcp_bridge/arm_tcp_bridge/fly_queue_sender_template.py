import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_tcp_bridge.command_builder import (
    add_fly_circular,
    add_fly_linear,
    clear_fly_queue,
    execute_fly_queue,
    set_acceleration,
    set_deceleration,
    set_fly_cart,
    set_joint_speed,
    set_linear_speed,
)
from arm_tcp_bridge_interfaces.action import ExecuteCommand


class FlyQueueSender(Node):
    def __init__(self) -> None:
        super().__init__("fly_queue_sender")

        self.declare_parameter(
            "action_name",
            "/arm/execute",
        )

        action_name = str(
            self.get_parameter("action_name").value
        )

        self._client = ActionClient(
            self,
            ExecuteCommand,
            action_name,
        )

        self.get_logger().info(
            f"Using arm action server: {action_name}"
        )

    def send_command(
        self,
        command: str,
    ) -> bool:
        self.get_logger().info(
            f"Sending: {command}"
        )

        goal = ExecuteCommand.Goal()
        goal.command = command

        self._client.wait_for_server()

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(
                f"No goal handle returned: {command}"
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Rejected: {command}"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error(
                f"No result returned: {command}"
            )
            return False

        result = result_response.result
        if not result.success:
            self.get_logger().error(
                f"Failed: {command}; message={result.message}"
            )
            return False

        self.get_logger().info(
            f"Done: {result.message}"
        )
        return True

    def _feedback_callback(
        self,
        feedback_msg,
    ) -> None:
        self.get_logger().info(
            f"Feedback: {feedback_msg.feedback.state}"
        )


def build_commands() -> list[str]:
    return [
        set_joint_speed(5),
        set_linear_speed(0.05),
        set_acceleration(10),
        set_deceleration(10),
        set_fly_cart(
            stress_percent=10,
            trajectory_mode=0,
            fly_distance_mm=5,
        ),
        clear_fly_queue(),
        add_fly_linear(
            [
                -1974.069,
                -353.247,
                955.513,
                -169.903,
                48.381,
                -46.638,
            ]
        ),
        add_fly_circular(
            [
                -1969.069,
                -353.247,
                960.513,
                -169.903,
                48.381,
                -46.638,
            ],
            [
                -1964.069,
                -353.247,
                955.513,
                -169.903,
                48.381,
                -46.638,
            ],
        ),
        add_fly_linear(
            [
                -1954.069,
                -353.247,
                955.513,
                -169.903,
                48.381,
                -46.638,
            ]
        ),
        execute_fly_queue(),
    ]


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FlyQueueSender()

    try:
        commands = build_commands()

        for command in commands:
            if not node.send_command(command):
                node.get_logger().error(
                    "Command sequence stopped"
                )
                break
        else:
            node.get_logger().info(
                "Command sequence finished"
            )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
