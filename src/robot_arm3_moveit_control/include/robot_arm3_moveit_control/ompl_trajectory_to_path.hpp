#pragma once

#include <cstddef>

#include <arm_tcp_bridge_interfaces/msg/path_block.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace robot_arm3_moveit_control
{
struct TrajectoryPathOptions
{
  double coupling_offset{ 1.5708 };
  double joint_3_min{ -4.0317 };
  double joint_3_max{ 0.0 };
  double joint_7_min{ -1.151917306 };
  double joint_7_max{ 1.047197551 };
  double duplicate_threshold{ 1e-6 };
  double segment_override{ 5.0 };
  std::size_t max_nodes{ 1000 };
  std::uint8_t termination_type{ 3 };
  // MoveIt trajectories include their current state as the first sample.
  // PATH nodes are destinations, so callers that publish an OMPL path can
  // omit that non-motion sample while preserving expected_start_deg.
  bool omit_initial_node{ false };
};

arm_tcp_bridge_interfaces::msg::PathBlock trajectory_to_joint_path(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const TrajectoryPathOptions& options);
}  // namespace robot_arm3_moveit_control
