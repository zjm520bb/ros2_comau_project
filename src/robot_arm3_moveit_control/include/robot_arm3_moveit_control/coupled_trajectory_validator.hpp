#pragma once

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace robot_arm3_moveit_control
{

void validate_coupled_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory, double coupling_offset,
                                 double joint_3_min, double joint_3_max, double joint_7_min, double joint_7_max);

}  // namespace robot_arm3_moveit_control
