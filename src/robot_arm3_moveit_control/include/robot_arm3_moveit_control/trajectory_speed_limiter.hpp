#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <array>
#include <string>

namespace robot_arm3_moveit_control
{

struct MotionSpeedSettings
{
  std::array<double, 6> joint_overrides_percent{};
  std::array<double, 6> max_joint_velocities{};
  std::array<double, 6> max_joint_accelerations{};
  double linear_speed_mps{ 0.05 };
  double acceleration_percent{ 10.0 };
  double deceleration_percent{ 10.0 };
};

struct ScalingResult
{
  double time_scale{ 1.0 };
  std::string limiting_reason{ "none" };
};

void validate_speed_settings(const MotionSpeedSettings& settings);

ScalingResult apply_speed_limits(moveit_msgs::msg::RobotTrajectory& trajectory,
                                 const moveit::core::RobotModelConstPtr& robot_model,
                                 const std::string& end_effector_link, const MotionSpeedSettings& settings,
                                 bool enforce_linear_speed);

}  // namespace robot_arm3_moveit_control
