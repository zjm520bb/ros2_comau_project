import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_tcp_bridge.path_sequence_io import load_sequence
from arm_tcp_bridge_interfaces.action import ExecutePathSequence


class SequenceSender(Node):
    def __init__(self, args) -> None:
        super().__init__("path_sequence_sender")
        self._args = args
        self._client = ActionClient(
            self,
            ExecutePathSequence,
            args.action_name,
        )

    def run(self) -> int:
        sequence = load_sequence(self._args.path)
        goal = ExecutePathSequence.Goal()
        goal.sequence = sequence
        goal.require_start_check = not self._args.skip_start_check
        goal.start_tolerance_deg = [self._args.start_tolerance_deg] * 6
        goal.feedback_timeout_s = self._args.feedback_timeout_s
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("PATH sequence action server is unavailable")
        future = self._client.send_goal_async(
            goal,
            feedback_callback=lambda message: self.get_logger().info(
                f"sequence={message.feedback.state}; "
                f"path={message.feedback.current_path}/"
                f"{message.feedback.total_paths}; "
                f"node={message.feedback.current_node}"
            ),
        )
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("PATH sequence goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(result.message)
        return 0 if result.success else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument(
        "--action-name",
        default="/arm/execute_path_sequence",
    )
    parser.add_argument("--start-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--feedback-timeout-s", type=float, default=0.5)
    parser.add_argument("--skip-start-check", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = SequenceSender(parsed)
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
