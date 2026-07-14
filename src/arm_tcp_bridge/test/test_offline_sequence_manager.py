import rclpy
from std_srvs.srv import SetBool, Trigger

from arm_tcp_bridge.offline_sequence_manager import OfflineSequenceManager
from arm_tcp_bridge_interfaces.msg import PathBlock, PathNode


def _block(name, start, end):
    block = PathBlock()
    block.name = name
    block.path_type = 0
    block.expected_start_deg = [start, 0.0, -90.0, 0.0, 0.0, 0.0]
    block.expected_end_deg = [end, 0.0, -90.0, 0.0, 0.0, 0.0]
    block.start_index = 1
    block.end_index = 1
    node = PathNode()
    node.motion_type = PathNode.LINEAR
    node.target = [end, 0.0, 1000.0, 0.0, 90.0, 0.0]
    node.segment_override = 5.0
    node.segment_data = True
    node.fly = True
    node.fly_percent = 50.0
    block.nodes = [node]
    return block


def test_recording_gate_wait_edit_and_compatible_merge():
    rclpy.init()
    node = OfflineSequenceManager()
    try:
        node._draft_callback(_block("ignored", 0.0, 1.0))
        assert node._draft is None

        response = node._start_recording(
            Trigger.Request(),
            Trigger.Response(),
        )
        assert response.success

        node._draft_callback(_block("first", 0.0, 1.0))
        wait_request = SetBool.Request()
        wait_request.data = True
        wait_response = node._set_draft_end_wait(
            wait_request,
            SetBool.Response(),
        )
        assert wait_response.success
        assert node._draft.nodes[-1].wait
        assert not node._draft.nodes[-1].fly
        assert node._accept(
            Trigger.Request(),
            Trigger.Response(),
        ).success

        node._draft_callback(_block("second", 1.0, 2.0))
        assert node._accept(
            Trigger.Request(),
            Trigger.Response(),
        ).success
        assert len(node._paths) == 1
        assert len(node._paths[0].nodes) == 2
        assert node._paths[0].nodes[0].wait

        response = node._stop_recording(
            Trigger.Request(),
            Trigger.Response(),
        )
        assert response.success
    finally:
        node.destroy_node()
        rclpy.shutdown()
