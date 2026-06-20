#include <gtest/gtest.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_arm3_moveit_control/coupled_trajectory_validator.hpp"
#include "robot_arm3_moveit_control/wrist_turn_preserver.hpp"

namespace control = robot_arm3_moveit_control;

constexpr double kPi = 3.14159265358979323846;

TEST(WristTurnPreserver, KeepsTheCurrentAbsoluteTurn)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "joint_6" };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 0.813620 };
  trajectory.joint_trajectory.points.push_back(point);

  control::preserve_absolute_turns(trajectory, "joint_6", -43.168, -47.12, 47.12);

  EXPECT_NEAR(trajectory.joint_trajectory.points[0].positions[0], 0.813620 - 7.0 * 2.0 * kPi, 1e-9);
}

TEST(CoupledTrajectoryValidator, RejectsIntermediateJointThreeViolation)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = { "joint_2", "joint_7" };
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { -1.3, 0.4 };
  trajectory.joint_trajectory.points.push_back(point);

  EXPECT_THROW(control::validate_coupled_trajectory(trajectory, 1.5708, -4.0317, 0.0, -1.151917306,
                                                    1.047197551),
               std::runtime_error);
}
