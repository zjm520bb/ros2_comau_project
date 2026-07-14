#pragma once

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace robot_arm3_moveit_control
{

moveit_msgs::msg::Constraints make_wrist_corridor_constraints(
    double joint_4_start, double joint_4_target,
    double joint_6_start, double joint_6_target,
    double margin_radians);

void validate_wrist_corridor_trajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const moveit_msgs::msg::Constraints& constraints);

}  // namespace robot_arm3_moveit_control
