#pragma once

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <string>

namespace robot_arm3_moveit_control
{

void preserve_absolute_turns(moveit_msgs::msg::RobotTrajectory& trajectory, const std::string& joint_name,
                             double current_position, double minimum_position, double maximum_position);

}  // namespace robot_arm3_moveit_control
