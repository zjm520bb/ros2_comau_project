#pragma once

#include "robot_arm3_moveit_control/path_model.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/motion_sequence_request.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_arm3_moveit_control
{

struct PathSequenceOptions
{
  std::string planning_group;
  std::string base_frame;
  std::string end_effector_link;
  std::string pipeline_id;
  std::string ptp_planner_id;
  std::string lin_planner_id;
  std::string circ_planner_id;
  Eigen::Isometry3d base_from_user{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d flange_to_default_tool{ Eigen::Isometry3d::Identity() };
  double planning_time{ 5.0 };
  int planning_attempts{ 10 };
  double pilz_max_trans_velocity{ 1.0 };
  double default_linear_speed{ 0.05 };
  double coupling_offset{ 1.5708 };
  double joint_3_min{ -4.0317 };
  double joint_3_max{ 0.0 };
  double joint_7_min{ -1.151917306 };
  double joint_7_max{ 1.047197551 };
  double normal_radius_safety_factor{ 0.45 };
};

moveit_msgs::msg::MotionSequenceRequest build_path_sequence_batch(
    const ExecutePath::Goal& goal, const std::vector<std::uint32_t>& node_indexes,
    const PathSequenceOptions& options, const moveit::core::RobotState& current_state,
    const moveit::core::RobotModelConstPtr& robot_model);

}  // namespace robot_arm3_moveit_control
