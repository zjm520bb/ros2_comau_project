#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_arm3_moveit_control/wrist_path_constraints.hpp"

namespace control = robot_arm3_moveit_control;

TEST(WristPathConstraints, LocksUnchangedJointsToTheConfiguredMargin)
{
  const auto constraints =
      control::make_wrist_corridor_constraints(1.0, 1.0, -2.0, -2.0, 0.01);

  ASSERT_EQ(constraints.joint_constraints.size(), 2u);
  EXPECT_EQ(constraints.joint_constraints[0].joint_name, "joint_4");
  EXPECT_NEAR(constraints.joint_constraints[0].position, 1.0, 1e-12);
  EXPECT_NEAR(constraints.joint_constraints[0].tolerance_below, 0.01, 1e-12);
  EXPECT_NEAR(constraints.joint_constraints[0].tolerance_above, 0.01, 1e-12);
  EXPECT_EQ(constraints.joint_constraints[1].joint_name, "joint_6");
  EXPECT_NEAR(constraints.joint_constraints[1].position, -2.0, 1e-12);
}

TEST(WristPathConstraints, CoversOnlyTheDirectIntervalBetweenEndpoints)
{
  const auto constraints =
      control::make_wrist_corridor_constraints(-1.0, 3.0, 2.0, -4.0, 0.1);

  const auto& joint_4 = constraints.joint_constraints[0];
  EXPECT_NEAR(joint_4.position, 1.0, 1e-12);
  EXPECT_NEAR(joint_4.position - joint_4.tolerance_below, -1.1, 1e-12);
  EXPECT_NEAR(joint_4.position + joint_4.tolerance_above, 3.1, 1e-12);

  const auto& joint_6 = constraints.joint_constraints[1];
  EXPECT_NEAR(joint_6.position, -1.0, 1e-12);
  EXPECT_NEAR(joint_6.position - joint_6.tolerance_below, -4.1, 1e-12);
  EXPECT_NEAR(joint_6.position + joint_6.tolerance_above, 2.1, 1e-12);
}

TEST(WristPathConstraints, RejectsInvalidInput)
{
  EXPECT_THROW(
      control::make_wrist_corridor_constraints(0.0, 0.0, 0.0, 0.0, 0.0),
      std::runtime_error);
  EXPECT_THROW(
      control::make_wrist_corridor_constraints(
          0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.01),
      std::runtime_error);
}

TEST(WristPathConstraints, ValidatesTheReturnedTrajectoryBeforeExecution)
{
  const auto constraints =
      control::make_wrist_corridor_constraints(0.0, 0.0, 1.0, 2.0, 0.01);
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "joint_4", "joint_6" };

  trajectory_msgs::msg::JointTrajectoryPoint valid_point;
  valid_point.positions = { 0.005, 1.5 };
  trajectory.joint_trajectory.points.push_back(valid_point);
  EXPECT_NO_THROW(
      control::validate_wrist_corridor_trajectory(trajectory, constraints));

  trajectory.joint_trajectory.points[0].positions[0] = 0.02;
  EXPECT_THROW(
      control::validate_wrist_corridor_trajectory(trajectory, constraints),
      std::runtime_error);
}
