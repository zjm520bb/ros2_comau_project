cmake_minimum_required(VERSION 3.8)

project(robot_arm3)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes textures urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(PROGRAMS
	scripts/passive_joint_solver_node.py
	scripts/gazebo_passive_joint_controller.py
	scripts/gazebo_initial_hold_publisher.py
	scripts/passive_joint_solver_node_03.py
	scripts/gazebo_passive_joint_controller_03.py
	scripts/gazebo_initial_hold_publisher_03.py
	scripts/passive_joint_solver_node_05.py
	scripts/gazebo_passive_joint_controller_05.py
	scripts/gazebo_initial_hold_publisher_05.py
	scripts/gazebo_passive_joint_controller_06.py
	scripts/gazebo_initial_hold_publisher_06.py
	scripts/c4g_gazebo_sync_node.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
