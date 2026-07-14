cmake_minimum_required(VERSION 3.8)

project(robot_arm3)

find_package(ament_cmake REQUIRED)
find_package(arm_tcp_bridge_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-utils1 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(robot_arm3_joint_reset_system SHARED
  src/joint_reset_system.cpp)
target_link_libraries(robot_arm3_joint_reset_system
  ignition-gazebo6::core
  ignition-utils1::core
  gz-cmake3::utilities)
ament_target_dependencies(robot_arm3_joint_reset_system
  arm_tcp_bridge_interfaces
  controller_manager_msgs
  rclcpp)

foreach(dir config launch meshes textures urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(PROGRAMS
	scripts/passive_joint_solver_node_05.py
	scripts/gazebo_passive_joint_controller_06.py
	scripts/gazebo_initial_hold_publisher_06.py
	scripts/c4g_gazebo_sync_node.py
	scripts/moveit_gazebo_trajectory_bridge.py
	scripts/moveit_environment_boxes.py
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_arm3_joint_reset_system
	LIBRARY DESTINATION lib)

if(BUILD_TESTING)
	find_package(ament_cmake_pytest REQUIRED)
	ament_add_pytest_test(
		test_moveit_gazebo_trajectory_bridge
		test/test_moveit_gazebo_trajectory_bridge.py
	)
	ament_add_pytest_test(
		test_moveit_environment_boxes
		test/test_moveit_environment_boxes.py
	)
endif()

ament_package()
