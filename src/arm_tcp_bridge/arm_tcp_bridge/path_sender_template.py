import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_tcp_bridge_interfaces.action import ExecutePath
from arm_tcp_bridge_interfaces.msg import (
    PathCondition,
    PathNode,
)


def _node(
    target: list[float],
    fly: bool,
    condition_mask: int,
) -> PathNode:
    node = PathNode()
    node.motion_type = PathNode.JOINT
    node.target = target
    node.linear_speed = 0.05
    node.rotational_speed = 10.0
    node.segment_override = 5.0
    node.termination_type = 1
    node.tolerance = 1.0
    node.segment_data = True
    node.fly = fly
    node.fly_type = 0
    node.fly_percent = 75.0
    node.fly_distance_mm = 5.0
    node.fly_trajectory = 0
    node.stress_percent = 10.0
    node.condition_mask = condition_mask
    node.condition_mask_back = condition_mask
    return node


def build_goal(
    enable_conditions: bool = False,
    wait_at_node: int = 0,
) -> ExecutePath.Goal:
    current_joints = [0.0, 0.0, -90.0, 0.0, 0.0, 0.0]
    joint_1_plus_1_degree = [
        10.0,
        0.0,
        -90.0,
        0.0,
        0.0,
        0.0,
    ]

    goal = ExecutePath.Goal()
    goal.path_id = 1
    goal.path_type = ExecutePath.Goal.JOINT
    goal.start_index = 1
    goal.end_index = 3

    condition_mask = 0
    if enable_conditions:
        start = PathCondition()
        start.slot = 1
        start.handler_id = 10
        end = PathCondition()
        end.slot = 2
        end.handler_id = 11
        goal.conditions = [start, end]
        condition_mask = 3
    else:
        goal.conditions = []

    goal.nodes = [
        _node(
            current_joints,
            False,
            condition_mask,
        ),
        _node(
            joint_1_plus_1_degree,
            False,
            condition_mask,
        ),
        _node(
            current_joints,
            False,
            condition_mask,
        ),
    ]
    if wait_at_node not in (0, 1, 2, 3):
        raise ValueError("wait_at_node must be within 0..3")
    if wait_at_node:
        goal.nodes[wait_at_node - 1].wait = True
    return goal


class PathSender(Node):
    def __init__(self) -> None:
        super().__init__("path_sender_template")
        self.declare_parameter("path_action_name", "/sim/arm/execute_path")
        self.declare_parameter("enable_conditions", False)
        self.declare_parameter("wait_at_node", 0)
        self._client = ActionClient(
            self,
            ExecutePath,
            str(self.get_parameter("path_action_name").value),
        )

    def run(self) -> None:
        self._client.wait_for_server()
        send_future = self._client.send_goal_async(
            build_goal(
                bool(
                    self.get_parameter(
                        "enable_conditions"
                    ).value
                ),
                int(
                    self.get_parameter(
                        "wait_at_node"
                    ).value
                ),
            ),
            feedback_callback=lambda msg: self.get_logger().info(
                f"PATH feedback: {msg.feedback.state}; "
                f"uploaded={msg.feedback.uploaded_nodes}; "
                f"node={msg.feedback.current_node}; "
                f"waiting={msg.feedback.waiting}"
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("PATH goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()
        if response is None:
            raise RuntimeError("PATH action returned no result")
        self.get_logger().info(response.result.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathSender()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
