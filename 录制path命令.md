录制PATH
cd /home/fishros/ros2_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

离线规划：
ros2 launch robot_arm3_moveit_config offline_path_planning.launch.py \
  use_rviz:=true \
  start_c4g_bridge:=false \
  export_path:=/home/fishros/ros2_robot_ws/path_programs/program_01.c4gseq.yaml

清空原序列，并关闭记录
ros2 service call \
  /offline_path_sequence_manager/clear \
  std_srvs/srv/Trigger

查询状态
ros2 service call \
  /offline_path_sequence_manager/get_status \
  arm_tcp_bridge_interfaces/srv/GetOfflineSequenceStatus

开始记录
ros2 service call \
  /offline_path_sequence_manager/start_recording \
  std_srvs/srv/Trigger

内部等待节点
ros2 service call \
  /offline_path_sequence_manager/set_draft_end_wait \
  std_srvs/srv/SetBool \
  "{data: true}"

PATH块之间等待
ros2 service call \
  /offline_path_sequence_manager/set_draft_wait_after \
  std_srvs/srv/SetBool \
  "{data: true}"
然后接受

接受
ros2 service call \
  /offline_path_sequence_manager/accept_draft \
  std_srvs/srv/Trigger

拒绝
ros2 service call \
  /offline_path_sequence_manager/reject_draft \
  std_srvs/srv/Trigger

撤销上一条path
ros2 service call \
  /offline_path_sequence_manager/undo_last \
  std_srvs/srv/Trigger

结束记录
ros2 service call \
  /offline_path_sequence_manager/stop_recording \
  std_srvs/srv/Trigger

完整Gazebo预演
ros2 service call \
  /offline_path_sequence_manager/preview_all \
  std_srvs/srv/Trigger

修改导出位置和文件名
ros2 param set \
  /offline_path_sequence_manager \
  export_path \
  /home/fishros/ros2_robot_ws/path_programs/program_02.c4gseq.yaml

导出离线文件
ros2 service call \
  /offline_path_sequence_manager/export \
  std_srvs/srv/Trigger

检查：
sed -n '1,240p' /home/fishros/ros2_robot_ws/path_programs/program_01.c4gseq.yaml

发送离线文件
ros2 launch robot_arm3 gazebo_mirror_c4g_boxes.launch.py \
  robot_ip:=130.149.138.38 \
  enable_motion_control:=true \
  enable_path_protocol:=true \
  c4g_protocol_version:=2
另一个终端：
source /home/fishros/ros2_robot_ws/install/setup.bash

ros2 run arm_tcp_bridge send_path_sequence \
  /home/fishros/ros2_robot_ws/path_programs/program_01.c4gseq.yaml \
  --start-tolerance-deg 0.1

直接发送C4G
ros2 launch robot_arm3_moveit_config offline_path_planning.launch.py \
  use_rviz:=true \
  start_c4g_bridge:=true \
  robot_ip:=130.149.138.38

发送
ros2 service call \
  /offline_path_sequence_manager/send \
  std_srvs/srv/Trigger