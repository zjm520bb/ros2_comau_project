#include <gtest/gtest.h>

#include <cmath>

#include "robot_arm3_moveit_control/ompl_trajectory_to_path.hpp"

namespace control = robot_arm3_moveit_control;

TEST(OmplTrajectoryToPath, ConvertsCoupledJointAndDegrees)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {
    "joint_4", "joint_1", "joint_7", "joint_2", "joint_6", "joint_5"
  };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 0.4, 0.1, 0.3, -0.2, 0.6, 0.5 };
  trajectory.joint_trajectory.points.push_back(point);

  const auto path = control::trajectory_to_joint_path(
      trajectory, control::TrajectoryPathOptions{});
  ASSERT_EQ(path.nodes.size(), 1u);
  constexpr double scale = 180.0 / 3.14159265358979323846;
  EXPECT_NEAR(path.nodes[0].target[0], 0.1 * scale, 1e-9);
  EXPECT_NEAR(path.nodes[0].target[1], -0.2 * scale, 1e-9);
  EXPECT_NEAR(path.nodes[0].target[2], (0.3 + 0.2 - 1.5708) * scale, 1e-9);
  EXPECT_FALSE(path.nodes[0].fly);
}

TEST(OmplTrajectoryToPath, RejectsNodeLimit)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {
    "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6"
  };
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  trajectory_msgs::msg::JointTrajectoryPoint second = first;
  second.positions[0] = 0.1;
  trajectory.joint_trajectory.points = { first, second };
  control::TrajectoryPathOptions options;
  options.max_nodes = 1;
  EXPECT_THROW(control::trajectory_to_joint_path(trajectory, options), std::runtime_error);
}

TEST(OmplTrajectoryToPath, OmitsInitialStateFromExecutableNodes)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {
    "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6"
  };
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = { 0.1, -0.2, 0.3, 0.4, 0.5, 0.6 };
  trajectory_msgs::msg::JointTrajectoryPoint second = first;
  second.positions[0] = 0.2;
  trajectory.joint_trajectory.points = { first, second };

  control::TrajectoryPathOptions options;
  options.omit_initial_node = true;
  options.max_nodes = 1;
  const auto path = control::trajectory_to_joint_path(trajectory, options);

  constexpr double scale = 180.0 / 3.14159265358979323846;
  ASSERT_EQ(path.nodes.size(), 1u);
  EXPECT_EQ(path.start_index, 1u);
  EXPECT_EQ(path.end_index, 1u);
  EXPECT_NEAR(path.expected_start_deg[0], 0.1 * scale, 1e-9);
  EXPECT_NEAR(path.expected_end_deg[0], 0.2 * scale, 1e-9);
  EXPECT_NEAR(path.nodes[0].target[0], 0.2 * scale, 1e-9);
}
