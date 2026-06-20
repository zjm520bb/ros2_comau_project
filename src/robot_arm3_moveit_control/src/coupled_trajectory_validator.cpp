#include "robot_arm3_moveit_control/coupled_trajectory_validator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_arm3_moveit_control
{

void validate_coupled_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory, double coupling_offset,
                                 double joint_3_min, double joint_3_max, double joint_7_min, double joint_7_max)
{
  const auto& names = trajectory.joint_trajectory.joint_names;
  const auto joint_2_found = std::find(names.begin(), names.end(), "joint_2");
  const auto joint_7_found = std::find(names.begin(), names.end(), "joint_7");
  if (joint_2_found == names.end() || joint_7_found == names.end())
    throw std::runtime_error("Trajectory does not contain joint_2 and joint_7");
  const std::size_t joint_2_index = std::distance(names.begin(), joint_2_found);
  const std::size_t joint_7_index = std::distance(names.begin(), joint_7_found);

  for (std::size_t point_index = 0; point_index < trajectory.joint_trajectory.points.size(); ++point_index)
  {
    const auto& positions = trajectory.joint_trajectory.points[point_index].positions;
    if (joint_2_index >= positions.size() || joint_7_index >= positions.size())
      throw std::runtime_error("Trajectory point has an incomplete position vector");
    const double joint_2 = positions[joint_2_index];
    const double joint_7 = positions[joint_7_index];
    const double joint_3 = joint_7 - joint_2 - coupling_offset;
    if (!std::isfinite(joint_2) || !std::isfinite(joint_7) || joint_7 < joint_7_min || joint_7 > joint_7_max)
      throw std::runtime_error("Trajectory point " + std::to_string(point_index) + " violates joint_7 limits");
    if (joint_3 < joint_3_min || joint_3 > joint_3_max)
      throw std::runtime_error("Trajectory point " + std::to_string(point_index) + " violates coupled joint_3 limits");
  }
}

}  // namespace robot_arm3_moveit_control
