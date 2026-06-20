cmake_minimum_required(VERSION 3.8)
project(robot_arm3_moveit_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_tcp_bridge_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(command_parser src/command_parser.cpp)
target_include_directories(command_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(fly_queue src/fly_queue.cpp)
target_include_directories(fly_queue PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(trajectory_speed_limiter src/trajectory_speed_limiter.cpp)
target_include_directories(trajectory_speed_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_speed_limiter
  builtin_interfaces
  moveit_core
  moveit_msgs
  trajectory_msgs)

add_library(trajectory_postprocessing
  src/wrist_turn_preserver.cpp
  src/coupled_trajectory_validator.cpp)
target_include_directories(trajectory_postprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_postprocessing moveit_msgs)

add_library(pilz_sequence_builder src/pilz_sequence_builder.cpp)
target_include_directories(pilz_sequence_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pilz_sequence_builder fly_queue Eigen3::Eigen)
ament_target_dependencies(pilz_sequence_builder
  geometry_msgs
  moveit_core
  moveit_msgs)

add_executable(moveit_command_server src/moveit_command_server.cpp)
target_link_libraries(moveit_command_server command_parser fly_queue pilz_sequence_builder trajectory_speed_limiter trajectory_postprocessing Eigen3::Eigen)
ament_target_dependencies(moveit_command_server
  arm_tcp_bridge_interfaces
  builtin_interfaces
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action)

install(TARGETS command_parser fly_queue pilz_sequence_builder trajectory_speed_limiter trajectory_postprocessing moveit_command_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_parser test/test_command_parser.cpp)
  target_link_libraries(test_command_parser command_parser)
  ament_add_gtest(test_trajectory_speed_limiter test/test_trajectory_speed_limiter.cpp)
  target_link_libraries(test_trajectory_speed_limiter trajectory_speed_limiter)
  ament_target_dependencies(test_trajectory_speed_limiter moveit_msgs trajectory_msgs)
  ament_add_gtest(test_wrist_and_coupling test/test_wrist_and_coupling.cpp)
  target_link_libraries(test_wrist_and_coupling trajectory_postprocessing)
  ament_target_dependencies(test_wrist_and_coupling moveit_msgs trajectory_msgs)
  ament_add_gtest(test_fly_queue test/test_fly_queue.cpp)
  target_link_libraries(test_fly_queue fly_queue pilz_sequence_builder Eigen3::Eigen)
  ament_target_dependencies(test_fly_queue moveit_msgs)
endif()

ament_package()
