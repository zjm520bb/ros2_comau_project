#include "robot_arm3_moveit_control/wrist_turn_preserver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robot_arm3_moveit_control
{

void preserve_absolute_turns(moveit_msgs::msg::RobotTrajectory& trajectory, const std::string& joint_name,
                             double current_position, double minimum_position, double maximum_position)
{
  constexpr double two_pi = 6.28318530717958647692;
  const auto found = std::find(trajectory.joint_trajectory.joint_names.begin(),
                               trajectory.joint_trajectory.joint_names.end(), joint_name);
  if (found == trajectory.joint_trajectory.joint_names.end())
    throw std::runtime_error("Trajectory is missing wrist joint " + joint_name);
  const std::size_t index = std::distance(trajectory.joint_trajectory.joint_names.begin(), found);

  double reference = current_position;
  for (auto& point : trajectory.joint_trajectory.points)
  {
    if (index >= point.positions.size())
      throw std::runtime_error("Trajectory point has an incomplete position vector");
    const double raw = point.positions[index];
    double best = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
    const long center_turn = std::lround((reference - raw) / two_pi);
    for (long turn = center_turn - 16; turn <= center_turn + 16; ++turn)
    {
      const double candidate = raw + static_cast<double>(turn) * two_pi;
      if (candidate < minimum_position || candidate > maximum_position)
        continue;
      const double distance = std::abs(candidate - reference);
      if (distance < best_distance)
      {
        best = candidate;
        best_distance = distance;
      }
    }
    if (!std::isfinite(best_distance))
      throw std::runtime_error("No in-limit absolute turn exists for " + joint_name);
    point.positions[index] = best;
    reference = best;
  }
}

}  // namespace robot_arm3_moveit_control
