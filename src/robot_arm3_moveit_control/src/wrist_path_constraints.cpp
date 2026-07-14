#include "robot_arm3_moveit_control/wrist_path_constraints.hpp"

#include <moveit_msgs/msg/joint_constraint.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_arm3_moveit_control
{
namespace
{

moveit_msgs::msg::JointConstraint make_corridor(
    const std::string& joint_name, double start, double target, double margin)
{
  if (!std::isfinite(start) || !std::isfinite(target))
    throw std::runtime_error("Wrist corridor endpoints must be finite");

  const double lower = std::min(start, target) - margin;
  const double upper = std::max(start, target) + margin;

  moveit_msgs::msg::JointConstraint constraint;
  constraint.joint_name = joint_name;
  constraint.position = 0.5 * (lower + upper);
  constraint.tolerance_below = constraint.position - lower;
  constraint.tolerance_above = upper - constraint.position;
  constraint.weight = 1.0;
  return constraint;
}

std::size_t joint_index(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& joint_name)
{
  const auto& names = trajectory.joint_trajectory.joint_names;
  const auto found = std::find(names.begin(), names.end(), joint_name);
  if (found == names.end())
    throw std::runtime_error("Trajectory is missing wrist joint " + joint_name);
  return static_cast<std::size_t>(std::distance(names.begin(), found));
}

}  // namespace

moveit_msgs::msg::Constraints make_wrist_corridor_constraints(
    double joint_4_start, double joint_4_target,
    double joint_6_start, double joint_6_target,
    double margin_radians)
{
  if (!std::isfinite(margin_radians) || margin_radians <= 0.0)
    throw std::runtime_error("Wrist corridor margin must be finite and greater than zero");

  moveit_msgs::msg::Constraints constraints;
  constraints.name = "ompl_wrist_corridor";
  constraints.joint_constraints.push_back(
      make_corridor("joint_4", joint_4_start, joint_4_target, margin_radians));
  constraints.joint_constraints.push_back(
      make_corridor("joint_6", joint_6_start, joint_6_target, margin_radians));
  return constraints;
}

void validate_wrist_corridor_trajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const moveit_msgs::msg::Constraints& constraints)
{
  for (const auto& constraint : constraints.joint_constraints)
  {
    if (constraint.joint_name != "joint_4" && constraint.joint_name != "joint_6")
      continue;

    const std::size_t index = joint_index(trajectory, constraint.joint_name);
    const double lower = constraint.position - constraint.tolerance_below;
    const double upper = constraint.position + constraint.tolerance_above;
    constexpr double epsilon = 1e-9;
    for (const auto& point : trajectory.joint_trajectory.points)
    {
      if (index >= point.positions.size())
        throw std::runtime_error("Trajectory point has an incomplete position vector");
      const double value = point.positions[index];
      if (!std::isfinite(value) || value < lower - epsilon || value > upper + epsilon)
        throw std::runtime_error(
            "Planned trajectory violates the " + constraint.joint_name + " wrist corridor");
    }
  }
}

}  // namespace robot_arm3_moveit_control
