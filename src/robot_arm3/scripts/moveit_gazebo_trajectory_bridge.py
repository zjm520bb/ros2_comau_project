#!/usr/bin/env python3

import copy
import math
import threading
from typing import Sequence

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


MOVEIT_JOINTS = (
    "joint_1",
    "joint_2",
    "joint_7",
    "joint_4",
    "joint_5",
    "joint_6",
)
GAZEBO_JOINTS = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)


class TrajectoryValidationError(ValueError):
    def __init__(self, message: str, error_code: int) -> None:
        super().__init__(message)
        self.error_code = error_code


def _duration_ns(duration) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _read_vector(
    values: Sequence[float],
    expected_size: int,
    field_name: str,
    *,
    required: bool = False,
) -> list[float]:
    if not values and not required:
        return []
    if len(values) != expected_size:
        raise TrajectoryValidationError(
            f"{field_name} must contain {expected_size} values, "
            f"received {len(values)}",
            FollowJointTrajectory.Result.INVALID_GOAL,
        )

    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise TrajectoryValidationError(
            f"{field_name} contains NaN or infinity",
            FollowJointTrajectory.Result.INVALID_GOAL,
        )
    return converted


def _moveit_to_gazebo_values(
    values: Sequence[float],
    index_by_name: dict[str, int],
    offset: float,
) -> list[float]:
    joint_2 = values[index_by_name["joint_2"]]
    joint_7 = values[index_by_name["joint_7"]]
    return [
        values[index_by_name["joint_1"]],
        joint_2,
        joint_7 - joint_2 - offset,
        values[index_by_name["joint_4"]],
        values[index_by_name["joint_5"]],
        values[index_by_name["joint_6"]],
    ]


def _gazebo_to_moveit_values(
    values: Sequence[float],
    index_by_name: dict[str, int],
    offset: float,
) -> list[float]:
    joint_2 = values[index_by_name["joint_2"]]
    joint_3 = values[index_by_name["joint_3"]]
    return [
        values[index_by_name["joint_1"]],
        joint_2,
        joint_2 + joint_3 + offset,
        values[index_by_name["joint_4"]],
        values[index_by_name["joint_5"]],
        values[index_by_name["joint_6"]],
    ]


def _transform_tolerances(
    tolerances: Sequence[JointTolerance],
) -> list[JointTolerance]:
    transformed = []
    for tolerance in tolerances:
        if tolerance.name not in MOVEIT_JOINTS:
            raise TrajectoryValidationError(
                f"Unsupported tolerance joint {tolerance.name!r}",
                FollowJointTrajectory.Result.INVALID_JOINTS,
            )
        converted = copy.deepcopy(tolerance)
        if converted.name == "joint_7":
            converted.name = "joint_3"
        transformed.append(converted)
    return transformed


def transform_goal(
    request: FollowJointTrajectory.Goal,
    *,
    coupling_offset: float,
    joint_7_min: float,
    joint_7_max: float,
    joint_3_min: float,
    joint_3_max: float,
    joint_3_max_velocity: float,
) -> FollowJointTrajectory.Goal:
    names = list(request.trajectory.joint_names)
    if len(names) != len(MOVEIT_JOINTS) or set(names) != set(MOVEIT_JOINTS):
        raise TrajectoryValidationError(
            "Expected trajectory joints "
            f"{list(MOVEIT_JOINTS)}, received {names}",
            FollowJointTrajectory.Result.INVALID_JOINTS,
        )
    if not request.trajectory.points:
        raise TrajectoryValidationError(
            "Trajectory contains no points",
            FollowJointTrajectory.Result.INVALID_GOAL,
        )
    if (
        request.multi_dof_trajectory.joint_names
        or request.multi_dof_trajectory.points
        or request.component_path_tolerance
        or request.component_goal_tolerance
    ):
        raise TrajectoryValidationError(
            "Multi-DOF trajectories and component tolerances "
            "are not supported",
            FollowJointTrajectory.Result.INVALID_GOAL,
        )

    index_by_name = {name: index for index, name in enumerate(names)}
    transformed = FollowJointTrajectory.Goal()
    transformed.trajectory.header = copy.deepcopy(request.trajectory.header)
    transformed.trajectory.joint_names = list(GAZEBO_JOINTS)

    previous_time_ns = -1
    for point_index, point in enumerate(request.trajectory.points):
        time_ns = _duration_ns(point.time_from_start)
        if time_ns < 0 or time_ns <= previous_time_ns:
            raise TrajectoryValidationError(
                "Trajectory point times must be non-negative "
                "and strictly increasing",
                FollowJointTrajectory.Result.INVALID_GOAL,
            )
        previous_time_ns = time_ns

        positions = _read_vector(
            point.positions,
            len(names),
            f"points[{point_index}].positions",
            required=True,
        )
        joint_7 = positions[index_by_name["joint_7"]]
        if not joint_7_min <= joint_7 <= joint_7_max:
            raise TrajectoryValidationError(
                f"points[{point_index}] has joint_7={joint_7:.6f}, "
                f"outside [{joint_7_min:.6f}, {joint_7_max:.6f}]",
                FollowJointTrajectory.Result.INVALID_GOAL,
            )
        velocities = _read_vector(
            point.velocities,
            len(names),
            f"points[{point_index}].velocities",
        )
        accelerations = _read_vector(
            point.accelerations,
            len(names),
            f"points[{point_index}].accelerations",
        )
        if point.effort:
            raise TrajectoryValidationError(
                "Effort trajectories cannot be mapped through "
                "the coupled joint",
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        output_point = JointTrajectoryPoint()
        output_point.positions = _moveit_to_gazebo_values(
            positions,
            index_by_name,
            coupling_offset,
        )
        joint_3 = output_point.positions[2]
        if not joint_3_min <= joint_3 <= joint_3_max:
            raise TrajectoryValidationError(
                f"points[{point_index}] converts to joint_3={joint_3:.6f}, "
                f"outside [{joint_3_min:.6f}, {joint_3_max:.6f}]",
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        if velocities:
            output_point.velocities = _moveit_to_gazebo_values(
                velocities,
                index_by_name,
                0.0,
            )
            joint_3_velocity = output_point.velocities[2]
            if (
                joint_3_max_velocity > 0.0
                and abs(joint_3_velocity) > joint_3_max_velocity
            ):
                raise TrajectoryValidationError(
                    f"points[{point_index}] converts to joint_3 velocity "
                    f"{joint_3_velocity:.6f}, exceeding "
                    f"{joint_3_max_velocity:.6f}",
                    FollowJointTrajectory.Result.INVALID_GOAL,
                )

        if accelerations:
            output_point.accelerations = _moveit_to_gazebo_values(
                accelerations,
                index_by_name,
                0.0,
            )
        output_point.time_from_start = copy.deepcopy(point.time_from_start)
        transformed.trajectory.points.append(output_point)

    transformed.path_tolerance = _transform_tolerances(
        request.path_tolerance
    )
    transformed.goal_tolerance = _transform_tolerances(
        request.goal_tolerance
    )
    transformed.goal_time_tolerance = copy.deepcopy(
        request.goal_time_tolerance
    )
    return transformed


def _transform_feedback_point(
    point: JointTrajectoryPoint,
    index_by_name: dict[str, int],
    offset: float,
) -> JointTrajectoryPoint:
    output = JointTrajectoryPoint()
    if point.positions:
        values = _read_vector(
            point.positions,
            len(GAZEBO_JOINTS),
            "feedback.positions",
        )
        output.positions = _gazebo_to_moveit_values(
            values,
            index_by_name,
            offset,
        )
    if point.velocities:
        values = _read_vector(
            point.velocities,
            len(GAZEBO_JOINTS),
            "feedback.velocities",
        )
        output.velocities = _gazebo_to_moveit_values(
            values,
            index_by_name,
            0.0,
        )
    if point.accelerations:
        values = _read_vector(
            point.accelerations,
            len(GAZEBO_JOINTS),
            "feedback.accelerations",
        )
        output.accelerations = _gazebo_to_moveit_values(
            values,
            index_by_name,
            0.0,
        )
    output.time_from_start = copy.deepcopy(point.time_from_start)
    return output


def transform_feedback(
    feedback: FollowJointTrajectory.Feedback,
    coupling_offset: float,
) -> FollowJointTrajectory.Feedback:
    names = list(feedback.joint_names)
    if len(names) != len(GAZEBO_JOINTS) or set(names) != set(GAZEBO_JOINTS):
        raise TrajectoryValidationError(
            f"Unexpected downstream feedback joints: {names}",
            FollowJointTrajectory.Result.INVALID_JOINTS,
        )

    index_by_name = {name: index for index, name in enumerate(names)}
    output = FollowJointTrajectory.Feedback()
    output.header = copy.deepcopy(feedback.header)
    output.joint_names = list(MOVEIT_JOINTS)
    output.desired = _transform_feedback_point(
        feedback.desired,
        index_by_name,
        coupling_offset,
    )
    output.actual = _transform_feedback_point(
        feedback.actual,
        index_by_name,
        coupling_offset,
    )
    # Error values are differences, so the fixed offset cancels out.
    output.error = _transform_feedback_point(
        feedback.error,
        index_by_name,
        0.0,
    )
    return output


class MoveItGazeboTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("moveit_gazebo_trajectory_bridge")

        self.declare_parameter(
            "input_action_name",
            "/moveit_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "output_action_name",
            "/arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("coupling_offset", 1.5708)
        self.declare_parameter("joint_7_min", -1.151917306)
        self.declare_parameter("joint_7_max", 1.047197551)
        self.declare_parameter("joint_3_min", -4.0317)
        self.declare_parameter("joint_3_max", 0.0)
        self.declare_parameter("joint_3_max_velocity", 1.0)
        self.declare_parameter("downstream_wait_timeout_s", 10.0)

        self._input_action_name = str(
            self.get_parameter("input_action_name").value
        )
        self._output_action_name = str(
            self.get_parameter("output_action_name").value
        )
        self._coupling_offset = float(
            self.get_parameter("coupling_offset").value
        )
        self._joint_7_min = float(self.get_parameter("joint_7_min").value)
        self._joint_7_max = float(self.get_parameter("joint_7_max").value)
        self._joint_3_min = float(self.get_parameter("joint_3_min").value)
        self._joint_3_max = float(self.get_parameter("joint_3_max").value)
        self._joint_3_max_velocity = float(
            self.get_parameter("joint_3_max_velocity").value
        )
        self._downstream_wait_timeout_s = float(
            self.get_parameter("downstream_wait_timeout_s").value
        )

        self._state_lock = threading.Lock()
        self._busy = False
        self._downstream_goal_handle = None
        self._callback_group = ReentrantCallbackGroup()

        self._downstream_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._output_action_name,
            callback_group=self._callback_group,
        )
        self._upstream_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._input_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "MoveIt-Gazebo trajectory bridge started: "
            f"{self._input_action_name} -> {self._output_action_name}"
        )

    def _convert_goal(
        self,
        request: FollowJointTrajectory.Goal,
    ) -> FollowJointTrajectory.Goal:
        return transform_goal(
            request,
            coupling_offset=self._coupling_offset,
            joint_7_min=self._joint_7_min,
            joint_7_max=self._joint_7_max,
            joint_3_min=self._joint_3_min,
            joint_3_max=self._joint_3_max,
            joint_3_max_velocity=self._joint_3_max_velocity,
        )

    def goal_callback(
        self,
        goal_request: FollowJointTrajectory.Goal,
    ) -> GoalResponse:
        try:
            self._convert_goal(goal_request)
        except TrajectoryValidationError as exc:
            self.get_logger().error(f"Rejecting trajectory: {exc}")
            return GoalResponse.REJECT

        with self._state_lock:
            if self._busy:
                self.get_logger().warn(
                    "Rejecting trajectory because bridge is busy"
                )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        with self._state_lock:
            downstream_goal_handle = self._downstream_goal_handle
        if downstream_goal_handle is not None:
            downstream_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_callback(
        self,
        goal_handle,
    ) -> FollowJointTrajectory.Result:
        try:
            transformed_goal = self._convert_goal(goal_handle.request)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Canceled before forwarding to Gazebo",
                )

            if not self._downstream_client.wait_for_server(
                timeout_sec=self._downstream_wait_timeout_s
            ):
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Downstream action unavailable: "
                    f"{self._output_action_name}",
                )

            send_future = self._downstream_client.send_goal_async(
                transformed_goal,
                feedback_callback=lambda message: self._forward_feedback(
                    goal_handle,
                    message.feedback,
                ),
            )
            downstream_goal_handle = await send_future
            if not downstream_goal_handle.accepted:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Gazebo controller rejected the converted trajectory",
                )

            with self._state_lock:
                self._downstream_goal_handle = downstream_goal_handle

            if goal_handle.is_cancel_requested:
                await downstream_goal_handle.cancel_goal_async()

            wrapped_result = await downstream_goal_handle.get_result_async()
            result = copy.deepcopy(wrapped_result.result)

            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif (
                wrapped_result.status == GoalStatus.STATUS_CANCELED
                or goal_handle.is_cancel_requested
            ):
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        except TrajectoryValidationError as exc:
            goal_handle.abort()
            return self._result(exc.error_code, str(exc))
        except Exception as exc:
            self.get_logger().error(f"Trajectory bridge failed: {exc}")
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"Bridge exception: {exc}",
            )
        finally:
            with self._state_lock:
                self._downstream_goal_handle = None
                self._busy = False

    def _forward_feedback(
        self,
        goal_handle,
        feedback: FollowJointTrajectory.Feedback,
    ) -> None:
        if not goal_handle.is_active:
            return
        try:
            goal_handle.publish_feedback(
                transform_feedback(feedback, self._coupling_offset)
            )
        except TrajectoryValidationError as exc:
            self.get_logger().warn(
                f"Dropping invalid downstream feedback: {exc}"
            )

    @staticmethod
    def _result(error_code: int, error_string: str):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    def destroy_node(self) -> None:
        self._upstream_server.destroy()
        self._downstream_client.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItGazeboTrajectoryBridge()
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


if __name__ == "__main__":
    main()
