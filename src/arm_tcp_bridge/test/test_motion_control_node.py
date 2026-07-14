from arm_tcp_bridge.motion_control_node import _path_state_fields


def test_path_state_fields_strip_c4g_numeric_padding():
    assert _path_state_fields("PATH_STATE: 1, 4, 2, 1") == [
        "1",
        "4",
        "2",
        "1",
    ]
