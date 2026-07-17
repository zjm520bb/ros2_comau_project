source /opt/ros/humble/setup.bash
source ~/ros2_robot_ws/install/setup.bash

ros2 launch arm_tcp_bridge \
  arm_tcp_bridge.launch.py \
  robot_ip:=130.149.138.38

运行 action server：
ros2 run arm_tcp_bridge action_server --ros-args \
  -p robot_ip:=130.149.138.38 \
  -p cmd_port:=8000

只gazebo
ros2 launch robot_arm3 gazebo_control_06.launch.py

C4G+Gazebo
ros2 launch robot_arm3 gazebo_mirror_c4g_boxes.launch.py robot_ip:=130.149.138.38

ros2 launch robot_arm3 gazebo_mirror_c4g_boxes.launch.py \
  robot_ip:=130.149.138.38 \
  enable_motion_control:=true \
  enable_path_protocol:=true \
  c4g_protocol_version:=2 \
  initial_sync_mode:=teleport

假反馈测试
ros2 launch robot_arm3 gazebo_mirror_c4g_boxes.launch.py start_arm_tcp_bridge:=false
ros2 topic pub /c4g/joint_states sensor_msgs/msg/JointState "
header:
  stamp: {sec: 0, nanosec: 0}
name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
position: [0.5, -0.5, -1.0, 0.6, 0.3, -0.5]
velocity: []
effort: []
" -r 20


ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'Hello'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'getPose'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'getJoints'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setBase:0,0,0,0,0,0'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setTool:0,0,100,0,0,0'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setUframe:0,0,0,0,0,0'}" \
  --feedback


关节速度设置：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setSpeedJnt:5'}" \
  --feedback

六轴独立速度设置：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setJointOverrides:5,6,7,8,9,10'}" \
  --feedback

线速度：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setSpeedLin:0.05'}" \
  --feedback

加速度和减速度：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setAcceleration:10'}" \
  --feedback

ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setDeceleration:10'}" \
  --feedback


平移0、1、2 是运动参考坐标系：0：BASE，1：TOOL，2：UFRAME
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'moveRelative:0,0,25,2'}" \
  --feedback

旋转
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'moveAbout:0,0,1,90,2'}" \
  --feedback

JOINT 运动：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'moveJoint:0,0,-90,0,0,0'}" \
  --feedback

到位判定方式：01234笛卡尔粗细，关节粗细，不等待稳定到位
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setTermination:1'}" \
  --feedback

末端姿态：0123 RS_WORLD RS_TRAJ EUL_WORLD WRIST_JNT
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setOrientation:0'}" \
  --feedback

LINEAR 运动：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'moveLin:-1984.069,-353.247,960.513,-169.903,48.381,-46.638'}" \
  --feedback

CIRCULAR 运动：
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'moveCircular:-1974.069,-353.247,957.513,-169.903,48.381,-46.638,-1964.069,-353.247,955.513,-169.903,48.381,-46.638'}" \
  --feedback

FLY 运动：
过渡程度
直线圆弧
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setFlyCart:10,0,5'}" \
  --feedback
关节点
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'setFlyNorm:10'}" \
  --feedback

清空旧队列
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'clearFlyQueue'}" \
  --feedback

加入点（直线-圆弧-直线）
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyLin:-1974.069,-353.247,955.513,-169.903,48.381,-46.638'}" \
  --feedback
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyCirc:-1969.069,-353.247,960.513,-169.903,48.381,-46.638,-1964.069,-353.247,955.513,-169.903,48.381,-46.638'}" \
  --feedback
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyLin:-1954.069,-353.247,955.513,-169.903,48.381,-46.638'}" \
  --feedback

加入点（关节）
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyJoint:170.905,47.031,-163.806,0.002,-115.425,-2473.363'}" \
  --feedback
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyJoint:168.905,47.031,-163.806,0.002,-115.425,-2473.363'}" \
  --feedback
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'addFlyJoint:169.905,47.031,-163.806,0.002,-115.425,-2473.363'}" \
  --feedback

执行
ros2 action send_goal \
  /arm/execute \
  arm_tcp_bridge_interfaces/action/ExecuteCommand \
  "{command: 'executeFlyQueue'}" \
  --feedback


PATH 运动（需要在 C4G 部署版本2 PDL程序）：

ros2 launch arm_tcp_bridge arm_tcp_bridge.launch.py \
  enable_motion_control:=true \
  enable_path_protocol:=true \
  c4g_protocol_version:=2

ros2 run arm_tcp_bridge send_path_template --ros-args \
  -p path_action_name:=/arm/execute_path

PATH 等待节点继续：

ros2 service call /arm/signal_path \
  arm_tcp_bridge_interfaces/srv/SignalPath \
  "{path_id: 1, expected_node: 2}"

PATH节间等待继续
ros2 service call /arm/signal_sequence \
  arm_tcp_bridge_interfaces/srv/SignalSequence \
  "{sequence_id: 1, expected_path: 1}"






启动 mock 外设
source ~/ros2_robot_ws/install/setup.bash
ros2 launch robot_arm3_peripherals peripherals_sim.launch.py

单独测试外设：
ros2 action send_goal /peripherals/motorspindel/execute \
  peripheral_interfaces/action/ExecutePeripheralCommand \
  "{device_id: 'motorspindel', command: 'set_speed', parameters: [{key: 'rpm', value: '3000'}]}" \
  --feedback

ros2 action send_goal /peripherals/motorspindel/execute \
  peripheral_interfaces/action/ExecutePeripheralCommand \
  "{device_id: 'motorspindel', command: 'start', parameters: []}" \
  --feedback

启动虚拟传感器
source ~/ros2_robot_ws/install/setup.bash
ros2 launch robot_arm3_sensors sensors_sim.launch.py

记录
ros2 launch robot_arm3_data_recording record_real.launch.py \
  output_root:=/home/fishros/ros2_robot_ws/records \
  output_name:=real_test_01

ros2 launch robot_arm3_data_recording record_sim.launch.py \
  output_root:=/home/fishros/ros2_robot_ws/records \
  output_name:=sim_test_01

分析
ros2 run robot_arm3_data_recording analyze_bag \
  /home/fishros/ros2_robot_ws/records/real_test_01



另一个终端

暂停当前运动
ros2 service call /arm/pause_motion std_srvs/srv/Trigger "{}"

继续暂停的运动
ros2 service call /arm/resume_motion std_srvs/srv/Trigger "{}"

中止当前运动
ros2 service call /arm/abort_motion std_srvs/srv/Trigger "{}"

节内等待继续
ros2 service call /arm/signal_path \
  arm_tcp_bridge_interfaces/srv/SignalPath \
  "{path_id: 100, expected_node: 1}"

节间等待继续
ros2 service call /arm/signal_sequence \
  arm_tcp_bridge_interfaces/srv/SignalSequence \
  "{sequence_id: 1, expected_path: 1}"



ros2 action send_goal   /arm/execute   arm_tcp_bridge_interfaces/action/ExecuteCommand   "{command: 'moveLin:569.920,0.142,0.002,17
9.997,90.001,-0.001'}"   --feedback