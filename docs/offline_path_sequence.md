# Offline MoveIt / C4G PATH workflow

Build and source the workspace, then start the offline editor:

```bash
ros2 launch robot_arm3_moveit_config offline_path_planning.launch.py
```

Set `start_c4g_bridge:=true` only when direct C4G execution is required.
Planning and Gazebo preview do not use C4G feedback.

Move the Gazebo robot to the desired offline start with any number of
`moveJoint`, `moveLin`, `moveCircular`, `moveJointAuto`, `movePoseAuto`, or
FLY commands. Recording is stopped by default, so none of these positioning
motions enters the PATH sequence.

Start a new recording session explicitly:

```bash
ros2 service call /offline_path_sequence_manager/clear std_srvs/srv/Trigger
ros2 service call /offline_path_sequence_manager/start_recording std_srvs/srv/Trigger
```

While recording, successful `/sim/arm/execute` motions generate drafts:

- `moveJoint`: one native JOINT node;
- `moveLin`, `moveRelative`, `moveAbout`: one native LINEAR node;
- `moveCircular`: one `SEG_VIA` plus one `CIRCULAR` node;
- `moveJointAuto`, `movePoseAuto`: sampled OMPL JOINT nodes;
- `executeFlyQueue`: one native JOINT or CARTESIAN PATH with FLY data.

Native `/sim/arm/execute_path` goals continue to publish their original JOINT
or CARTESIAN nodes. `path_sender_template.py` remains an unchanged standalone
single-PATH example; normal recording does not require editing it.

Review the Gazebo motion and accept or reject the draft:

```bash
ros2 service call /offline_path_sequence_manager/accept_draft std_srvs/srv/Trigger
ros2 service call /offline_path_sequence_manager/reject_draft std_srvs/srv/Trigger
```

Compatible adjacent drafts are merged. Consecutive LINEAR and CIRCULAR
commands therefore become one CARTESIAN PATH; an OMPL JOINT draft and a
CARTESIAN draft remain separate blocks.

Add a wait at the current draft endpoint before accepting it:

```bash
ros2 service call /offline_path_sequence_manager/set_draft_end_wait \
  std_srvs/srv/SetBool "{data: true}"
```

The next accepted draft must be compatible so the wait becomes an internal
node of the same PATH. A wait disables FLY on that endpoint. `SEG_VIA` cannot
wait.

Add a sequence-level wait after the current draft when the next movement will
be a different PATH type:

```bash
ros2 service call /offline_path_sequence_manager/set_draft_wait_after \
  std_srvs/srv/SetBool "{data: true}"
```

After accepting every draft, finish recording:

```bash
ros2 service call /offline_path_sequence_manager/stop_recording \
  std_srvs/srv/Trigger
```

Stopping is rejected while motion or a draft is active, or while a PATH ends
with a dangling internal node wait. Inspect the editor state with:

```bash
ros2 service call /offline_path_sequence_manager/get_status \
  arm_tcp_bridge_interfaces/srv/GetOfflineSequenceStatus
```

Other editing operations:

```bash
ros2 service call /offline_path_sequence_manager/undo_last std_srvs/srv/Trigger
ros2 service call /offline_path_sequence_manager/clear std_srvs/srv/Trigger
ros2 service call /offline_path_sequence_manager/preview_all std_srvs/srv/Trigger
```

When a complete preview waits inside a PATH, continue it with
`/sim/arm/signal_path`. When it waits between PATH blocks, continue with:

```bash
ros2 service call /sim/arm/signal_sequence \
  arm_tcp_bridge_interfaces/srv/SignalSequence \
  "{sequence_id: 1, expected_path: 1}"
```

For real execution use `/arm/signal_path` and `/arm/signal_sequence`.
Sequence-level continue always re-enters the normal per-block C4G start check
before the next PATH is uploaded.

Export the accepted sequence:

```bash
ros2 service call /offline_path_sequence_manager/export std_srvs/srv/Trigger
```

The default output is `/tmp/prepared_path.c4gseq.yaml`. It can later be sent
through the same validated sequence action:

```bash
ros2 run arm_tcp_bridge send_path_sequence \
  /tmp/prepared_path.c4gseq.yaml
```

For direct execution, start the offline launch with the C4G bridge and request
send:

```bash
ros2 launch robot_arm3_moveit_config offline_path_planning.launch.py \
  start_c4g_bridge:=true
ros2 service call /offline_path_sequence_manager/send std_srvs/srv/Trigger
```

Direct and file execution both require fresh C4G feedback and verify every
PATH block's six-axis `expected_start_deg` before it is uploaded. A mismatch
aborts the sequence; move the real robot to the reported start and send again.
YAML format version 2 stores `wait_after`; version-1 files remain readable and
default it to false.

The two C4G mirror launches default to instantaneous Gazebo joint reset:

```bash
ros2 launch robot_arm3 gazebo_mirror_c4g.launch.py
ros2 launch robot_arm3 gazebo_mirror_c4g_boxes.launch.py
```

Use `initial_sync_mode:=blend` only to restore the legacy visual transition.
