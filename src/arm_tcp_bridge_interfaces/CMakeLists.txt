cmake_minimum_required(VERSION 3.8)

project(arm_tcp_bridge_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/PathFrame.msg"
  "msg/PathCondition.msg"
  "msg/PathNode.msg"
  "msg/PathEvent.msg"
  "msg/PathBlock.msg"
  "msg/PathSequence.msg"
  "action/ExecuteCommand.action"
  "action/ExecutePath.action"
  "action/ExecutePathSequence.action"
  "srv/SignalPath.srv"
  "srv/GetPathState.srv"
  "srv/GetOfflineSequenceStatus.srv"
  "srv/ResetJoints.srv"
  "srv/SignalSequence.srv"
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
