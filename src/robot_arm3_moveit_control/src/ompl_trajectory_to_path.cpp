#include "robot_arm3_moveit_control/ompl_trajectory_to_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <arm_tcp_bridge_interfaces/action/execute_path.hpp>
#include <arm_tcp_bridge_interfaces/msg/path_node.hpp>

namespace robot_arm3_moveit_control
{
namespace
{
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
const std::array<std::string, 6> kMoveItJoints{
  "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6"
};

std::array<double, 6> c4g_axes(
    const trajectory_msgs::msg::JointTrajectoryPoint& point,
    const std::map<std::string, std::size_t>& indexes,
    const TrajectoryPathOptions& options)
{
  if (point.positions.size() != indexes.size())
    throw std::runtime_error("trajectory point position count does not match joint names");
  const double joint_2 = point.positions.at(indexes.at("joint_2"));
  const double joint_7 = point.positions.at(indexes.at("joint_7"));
  const double joint_3 = joint_7 - joint_2 - options.coupling_offset;
  if (joint_3 < options.joint_3_min || joint_3 > options.joint_3_max)
    throw std::runtime_error("trajectory produces joint_3 outside configured limits");
  if (joint_7 < options.joint_7_min || joint_7 > options.joint_7_max)
    throw std::runtime_error("trajectory produces joint_7 outside configured limits");
  std::array<double, 6> result{
    point.positions.at(indexes.at("joint_1")),
    joint_2,
    joint_3,
    point.positions.at(indexes.at("joint_4")),
    point.positions.at(indexes.at("joint_5")),
    point.positions.at(indexes.at("joint_6")),
  };
  for (auto& value : result)
  {
    if (!std::isfinite(value))
      throw std::runtime_error("trajectory contains NaN or infinity");
    value *= kRadiansToDegrees;
  }
  return result;
}

bool nearly_equal(const std::array<double, 6>& left,
                  const std::array<double, 6>& right,
                  double threshold)
{
  for (std::size_t index = 0; index < left.size(); ++index)
    if (std::abs(left[index] - right[index]) > threshold * kRadiansToDegrees)
      return false;
  return true;
}
}  // namespace

arm_tcp_bridge_interfaces::msg::PathBlock trajectory_to_joint_path(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const TrajectoryPathOptions& options)
{
  if (options.max_nodes == 0)
    throw std::runtime_error("max_nodes must be positive");
  if (options.segment_override <= 0.0 || options.segment_override > 100.0)
    throw std::runtime_error("segment_override must be within (0, 100]");
  const auto& joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty())
    throw std::runtime_error("cannot convert an empty trajectory");
  if (joint_trajectory.joint_names.size() != kMoveItJoints.size())
    throw std::runtime_error("trajectory must contain exactly six active joints");

  std::map<std::string, std::size_t> indexes;
  for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index)
    indexes[joint_trajectory.joint_names[index]] = index;
  for (const auto& name : kMoveItJoints)
    if (indexes.count(name) == 0)
      throw std::runtime_error("trajectory is missing active joint " + name);

  std::vector<std::array<double, 6>> axes;
  for (const auto& point : joint_trajectory.points)
  {
    const auto converted = c4g_axes(point, indexes, options);
    if (axes.empty() || !nearly_equal(axes.back(), converted, options.duplicate_threshold))
      axes.push_back(converted);
  }
  const std::size_t first_node = options.omit_initial_node ? 1 : 0;
  const std::size_t node_count = axes.size() - first_node;
  if (node_count > options.max_nodes)
    throw std::runtime_error("converted trajectory exceeds configured C4G PATH node limit");

  arm_tcp_bridge_interfaces::msg::PathBlock block;
  block.name = "ompl_joint_trajectory";
  block.path_type = arm_tcp_bridge_interfaces::action::ExecutePath::Goal::JOINT;
  block.start_index = 1;
  block.end_index = static_cast<std::uint32_t>(node_count);
  block.expected_start_deg = axes.front();
  block.expected_end_deg = axes.back();
  for (std::size_t index = first_node; index < axes.size(); ++index)
  {
    const auto& target = axes[index];
    arm_tcp_bridge_interfaces::msg::PathNode node;
    node.motion_type = arm_tcp_bridge_interfaces::msg::PathNode::JOINT;
    node.target = target;
    node.segment_override = options.segment_override;
    node.termination_type = options.termination_type;
    node.segment_data = true;
    node.fly = false;
    node.fly_type = 0;
    node.fly_trajectory = 0;
    block.nodes.push_back(node);
  }
  return block;
}
}  // namespace robot_arm3_moveit_control
