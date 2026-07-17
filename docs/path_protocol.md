# C4G PATH protocol

The ROS interface is `arm_tcp_bridge_interfaces/action/ExecutePath`. The real
robot bridge validates one complete action goal, then uploads it to C4G as
newline-delimited ASCII transactions. No transaction may exceed 254 bytes.

Upload order:

1. `beginPath:path_id,path_type,count,start,end`
2. zero or more `setPathFrame:path_id,index,x,y,z,e1,e2,e3`
3. zero or more `setPathCondition:path_id,slot,handler`
4. for every node, in order:
   - `pathNodeTarget:path_id,index,motion_type,p1,p2,p3,p4,p5,p6`
   - `pathNodeMotion:path_id,index,lin,rot,override,termination,tolerance,segment_data`
   - `pathNodeBlend:path_id,index,fly,type,percent,distance,trajectory,stress`
   - `pathNodeSync:path_id,index,reference,tool,mask,back_mask,wait`
   - `commitPathNode:path_id,index`
5. `commitPath:path_id,count`
6. `executePath:path_id,start,end`

Each transaction uses the legacy command handshake:

1. bridge sends the command;
2. C4G echoes it;
3. bridge sends `start`;
4. C4G returns `Movement finished`, `Movement unavailable`, `ERROR`, or
   `Motion aborted`.

During execution C4G may also send:

- `PATH_EVENT:path_id,node_index,handler`
- `PATH_WAITING:path_id,node_index`

`$SEG_WAIT` is resumed over the independent control connection:

`continuePath:path_id,expected_node`

Signals are not queued. A continue request is accepted only while the matching
PATH is waiting at the requested node.

Protocol version 1 is the legacy PDL implementation. Version 2 is the PATH
implementation. The bridge must be launched with both
`c4g_protocol_version:=2` and `enable_path_protocol:=true`; it never probes an
old controller with an unknown command.

Condition handlers currently reserved by the supplied PDL implementation are:

- 10: segment start;
- 11: segment end;
- 12: circular VIA.

Additional handler numbers require matching PDL `CONDITION` declarations and
matching simulation behavior.

The handlers are defined by `PDL2_ROS2_main_1`, not
`ROS2_MOVEMENTS_1`. C4G resolves a PATH `COND_TBL` entry in the context of the
program executing `MOVE ALONG`. `ROS2_MOVEMENTS_1` owns the PATH variables and
the global `SEGMENT WAIT` handlers, and therefore remains running with the
`DETACH` program attribute.

Every `beginPath` clears the native node arrays and all 32 entries of both
native condition tables. This prevents handlers uploaded by an earlier PATH
from leaking into a later PATH that does not use conditions.
