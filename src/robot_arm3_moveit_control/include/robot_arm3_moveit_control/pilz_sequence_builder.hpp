#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/motion_sequence_request.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

#include "robot_arm3_moveit_control/fly_queue.hpp"
#include "robot_arm3_moveit_control/trajectory_speed_limiter.hpp"

namespace robot_arm3_moveit_control
{

struct SequenceBuildOptions
{
  std::string planning_group;
  std::string base_frame;
  std::string end_effector_link;
  std::string pipeline_id;
  std::string ptp_planner_id;
  std::string lin_planner_id;
  std::string circ_planner_id;
  double planning_time{ 5.0 };
  int planning_attempts{ 1 };
  double pilz_max_trans_velocity{ 1.0 };
  double coupling_offset{ 1.5708 };
  double joint_3_min{ -4.0317 };
  double joint_3_max{ 0.0 };
  double joint_7_min{ -1.151917306 };
  double joint_7_max{ 1.047197551 };
  double normal_radius_safety_factor{ 0.45 };
};

std::vector<double> compute_fly_blend_radii(const std::vector<Eigen::Vector3d>& points,
                                            const FlySettings& settings, double normal_radius_safety_factor);

moveit_msgs::msg::MotionSequenceRequest build_pilz_sequence(
    const FlyQueue& queue, const FlySettings& fly_settings, const MotionSpeedSettings& speed_settings,
    const SequenceBuildOptions& options, const moveit::core::RobotState& current_state,
    const moveit::core::RobotModelConstPtr& robot_model);

}  // namespace robot_arm3_moveit_control
