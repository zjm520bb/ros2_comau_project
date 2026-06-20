import importlib.util
from pathlib import Path

import pytest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "moveit_gazebo_trajectory_bridge.py"
)
SPEC = importlib.util.spec_from_file_location("trajectory_bridge", SCRIPT_PATH)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


def _goal(positions, velocities=None):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [
        "joint_4",
        "joint_1",
        "joint_7",
        "joint_2",
        "joint_6",
        "joint_5",
    ]
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = velocities or []
    point.time_from_start.sec = 1
    goal.trajectory.points = [point]
    return goal


def _transform(goal):
    return BRIDGE.transform_goal(
        goal,
        coupling_offset=1.5708,
        joint_7_min=-1.151917306,
        joint_7_max=1.047197551,
        joint_3_min=-4.0317,
        joint_3_max=0.0,
        joint_3_max_velocity=1.0,
    )


def test_goal_conversion_reorders_joints_and_maps_coupling():
    output = _transform(
        _goal(
            positions=[0.4, 0.1, 0.3, -0.2, 0.6, 0.5],
            velocities=[0.04, 0.01, 0.03, -0.02, 0.06, 0.05],
        )
    )

    assert output.trajectory.joint_names == list(BRIDGE.GAZEBO_JOINTS)
    assert output.trajectory.points[0].positions == pytest.approx(
        [0.1, -0.2, -1.0708, 0.4, 0.5, 0.6]
    )
    assert output.trajectory.points[0].velocities == pytest.approx(
        [0.01, -0.02, 0.05, 0.04, 0.05, 0.06]
    )


def test_goal_conversion_rejects_joint_3_limit_violation():
    goal = _goal(positions=[0.4, 0.1, 3.0, -0.2, 0.6, 0.5])

    with pytest.raises(BRIDGE.TrajectoryValidationError, match="outside"):
        _transform(goal)


@pytest.mark.parametrize("joint_7", [-1.16, 1.05])
def test_goal_conversion_rejects_joint_7_limit_violation(joint_7):
    goal = _goal(
        positions=[0.4, 0.1, joint_7, -0.2, 0.6, 0.5]
    )

    with pytest.raises(
        BRIDGE.TrajectoryValidationError,
        match="joint_7=.*outside",
    ):
        _transform(goal)


def test_feedback_conversion_restores_moveit_joint_7():
    feedback = FollowJointTrajectory.Feedback()
    feedback.joint_names = list(BRIDGE.GAZEBO_JOINTS)
    feedback.actual.positions = [0.1, -0.2, -1.0708, 0.4, 0.5, 0.6]
    feedback.error.positions = [0.01, -0.02, 0.05, 0.04, 0.05, 0.06]

    output = BRIDGE.transform_feedback(feedback, 1.5708)

    assert output.joint_names == list(BRIDGE.MOVEIT_JOINTS)
    assert output.actual.positions == pytest.approx(
        [0.1, -0.2, 0.3, 0.4, 0.5, 0.6]
    )
    assert output.error.positions == pytest.approx(
        [0.01, -0.02, 0.03, 0.04, 0.05, 0.06]
    )
