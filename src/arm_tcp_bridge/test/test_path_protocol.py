import pytest

from arm_tcp_bridge.path_protocol import (
    PathValidationError,
    build_upload_commands,
    validate_path_goal,
)
from arm_tcp_bridge.path_sender_template import build_goal as build_template_goal
from arm_tcp_bridge_interfaces.action import ExecutePath
from arm_tcp_bridge_interfaces.msg import PathNode


def make_node(motion_type=PathNode.LINEAR):
    node = PathNode()
    node.motion_type = motion_type
    node.target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    node.linear_speed = 0.05
    node.rotational_speed = 10.0
    node.segment_override = 20.0
    node.termination_type = 1
    node.tolerance = 1.0
    node.segment_data = True
    node.fly = True
    node.fly_type = 1
    node.fly_percent = 75.0
    node.fly_distance_mm = 5.0
    node.fly_trajectory = 0
    node.stress_percent = 10.0
    return node


def make_goal():
    goal = ExecutePath.Goal()
    goal.path_id = 7
    goal.path_type = ExecutePath.Goal.CARTESIAN
    goal.nodes = [
        make_node(PathNode.LINEAR),
        make_node(PathNode.SEG_VIA),
        make_node(PathNode.CIRCULAR),
    ]
    return goal


def test_builds_bounded_incremental_commands():
    path = validate_path_goal(make_goal(), 20)
    commands = build_upload_commands(path)
    assert commands[0].startswith("beginPath:7,0,3")
    assert commands[-1] == "commitPath:7,3"
    assert sum(command.startswith("commitPathNode:") for command in commands) == 3
    assert max(len(command.encode("ascii")) for command in commands) <= 254


def test_rejects_circular_without_via():
    goal = make_goal()
    goal.nodes = [make_node(PathNode.CIRCULAR)]
    with pytest.raises(PathValidationError):
        validate_path_goal(goal, 20)


def test_rejects_joint_path_with_cartesian_node():
    goal = make_goal()
    goal.path_type = ExecutePath.Goal.JOINT
    goal.nodes = [make_node(PathNode.LINEAR)]
    with pytest.raises(PathValidationError):
        validate_path_goal(goal, 20)


def test_template_conditions_are_opt_in():
    basic = validate_path_goal(build_template_goal(False), 20)
    basic_commands = build_upload_commands(basic)
    assert basic.conditions == ()
    assert all(node.condition_mask == 0 for node in basic.nodes)
    assert not any(
        command.startswith("setPathCondition:")
        for command in basic_commands
    )

    with_events = validate_path_goal(build_template_goal(True), 20)
    event_commands = build_upload_commands(with_events)
    assert [condition.handler_id for condition in with_events.conditions] == [
        10,
        11,
    ]
    assert all(node.condition_mask == 3 for node in with_events.nodes)
    assert sum(
        command.startswith("setPathCondition:")
        for command in event_commands
    ) == 2


def test_template_wait_node_is_opt_in():
    without_wait = build_template_goal()
    assert not any(node.wait for node in without_wait.nodes)

    with_wait = build_template_goal(wait_at_node=2)
    assert [node.wait for node in with_wait.nodes] == [
        False,
        True,
        False,
    ]

    with pytest.raises(ValueError):
        build_template_goal(wait_at_node=4)
