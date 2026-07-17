# C4G PATH deployment

Deploy the four `_1` PDL programs as one matched set:

- `Connect_via_TCP_C4G_1`
- `ROS2_MOVEMENTS_1`
- `PDL2_ROS2_main_1`
- `ROS2_MOTION_COMMAND_1`

`ROS2_JOINT_FEEDBACK_1` is unchanged.

Compile `ROS2_MOVEMENTS_1` first because it exports PATH state and control
routines used by the other programs. Compile `PDL2_ROS2_main_1` after it
because the main program imports the PATH event reporting routine.

`ROS2_MOVEMENTS_1` is a persistent `DETACH` program. Start it first and leave
it running so that the dynamic PATH variables and the `SEGMENT WAIT`
condition handlers remain active. It must not attach arm 1.
`PDL2_ROS2_main_1` is the only PATH program that attaches arm 1. It defines
condition handlers 10, 11, and 12 because PDL resolves `COND_TBL` entries in
the context of the program executing `MOVE ALONG`.

Check controller memory after compiling the two dynamic PATH variables. The
repository cannot compile PDL locally, so the following must be verified on
C4G before enabling protocol version 2:

1. every predefined NODE field is supported by the installed C4G software;
2. `PATH_LEN`, `NODE_APP`, `NODE_DEL`, and variable node ranges compile;
3. `ROS2_MOVEMENTS_1` remains running without attaching arm 1;
4. `PDL2_ROS2_main_1` can attach arm 1 and resolve handlers 10, 11, and 12
   when `MOVE ALONG` runs through the exported `call_service` routine;
5. `WRITE` messages on the command LUN are LF-delimited;
6. `SIGNAL SEGMENT` can be invoked through the exported control routine;
7. controller stack and free dynamic memory are sufficient.

Keep the previous PDL files available for rollback. The new programs retain
the old ports, handshake, exported routines, commands, and terminal responses.
Old ROS clients therefore continue to work with the new PDL programs.

Launch the bridge only after the new PDL set is running:

```bash
ros2 launch arm_tcp_bridge arm_tcp_bridge.launch.py \
  enable_motion_control:=true \
  enable_path_protocol:=true \
  c4g_protocol_version:=2
```

Run the template without condition handlers first:

```bash
ros2 run arm_tcp_bridge send_path_template --ros-args \
  -p path_action_name:=/arm/execute_path
```

After the basic JOINT PATH succeeds, enable handlers 10 and 11:

```bash
ros2 run arm_tcp_bridge send_path_template --ros-args \
  -p path_action_name:=/arm/execute_path \
  -p enable_conditions:=true
```
