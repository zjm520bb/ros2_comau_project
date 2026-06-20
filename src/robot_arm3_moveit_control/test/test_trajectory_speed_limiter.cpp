#include <gtest/gtest.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <array>

#include "robot_arm3_moveit_control/trajectory_speed_limiter.hpp"

namespace control = robot_arm3_moveit_control;

namespace
{

control::MotionSpeedSettings settings()
{
  control::MotionSpeedSettings value;
  value.joint_overrides_percent.fill(100.0);
  value.max_joint_velocities.fill(1.0);
  value.max_joint_accelerations.fill(1.0);
  value.linear_speed_mps = 0.05;
  value.acceleration_percent = 100.0;
  value.deceleration_percent = 100.0;
  return value;
}

moveit_msgs::msg::RobotTrajectory trajectory()
{
  moveit_msgs::msg::RobotTrajectory result;
  result.joint_trajectory.joint_names = { "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6" };

  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  first.velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  first.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint second;
  second.positions = { 0.8, 0.0, 0.0, 0.0, 0.0, 0.0 };
  second.velocities = { 0.8, 0.0, 0.0, 0.0, 0.0, 0.0 };
  second.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  second.time_from_start.sec = 1;
  result.joint_trajectory.points = { first, second };
  return result;
}

}  // namespace

TEST(TrajectorySpeedLimiter, UniformOverrideStretchesTimeAndScalesVelocity)
{
  auto speed_settings = settings();
  speed_settings.joint_overrides_percent.fill(50.0);
  auto motion = trajectory();

  const auto result = control::apply_speed_limits(motion, nullptr, "", speed_settings, false);

  EXPECT_DOUBLE_EQ(result.time_scale, 1.6);
  EXPECT_EQ(motion.joint_trajectory.points[1].time_from_start.sec, 1);
  EXPECT_EQ(motion.joint_trajectory.points[1].time_from_start.nanosec, 600000000u);
  EXPECT_DOUBLE_EQ(motion.joint_trajectory.points[1].velocities[0], 0.5);
}

TEST(TrajectorySpeedLimiter, CoupledJointThreeCanBeTheLimitingAxis)
{
  auto speed_settings = settings();
  speed_settings.joint_overrides_percent[2] = 25.0;
  auto motion = trajectory();
  auto& point = motion.joint_trajectory.points[1];
  point.positions[1] = -0.4;
  point.positions[2] = 0.4;
  point.velocities[1] = -0.4;
  point.velocities[2] = 0.4;

  const auto result = control::apply_speed_limits(motion, nullptr, "", speed_settings, false);

  EXPECT_DOUBLE_EQ(result.time_scale, 3.2);
  EXPECT_EQ(result.limiting_reason, "physical joint_3 velocity");
}

TEST(TrajectorySpeedLimiter, AccelerationUsesAccelerationOverride)
{
  auto speed_settings = settings();
  speed_settings.acceleration_percent = 10.0;
  speed_settings.deceleration_percent = 50.0;
  auto motion = trajectory();
  motion.joint_trajectory.points[1].positions[0] = 0.0;
  motion.joint_trajectory.points[1].velocities[0] = 0.2;
  motion.joint_trajectory.points[1].accelerations[0] = 0.4;

  const auto result = control::apply_speed_limits(motion, nullptr, "", speed_settings, false);

  EXPECT_DOUBLE_EQ(result.time_scale, 2.0);
  EXPECT_EQ(result.limiting_reason, "physical joint_1 acceleration");
}

TEST(TrajectorySpeedLimiter, DecelerationUsesDecelerationOverride)
{
  auto speed_settings = settings();
  speed_settings.acceleration_percent = 50.0;
  speed_settings.deceleration_percent = 10.0;
  auto motion = trajectory();
  motion.joint_trajectory.points[1].positions[0] = 0.0;
  motion.joint_trajectory.points[1].velocities[0] = 0.2;
  motion.joint_trajectory.points[1].accelerations[0] = -0.4;

  const auto result = control::apply_speed_limits(motion, nullptr, "", speed_settings, false);

  EXPECT_DOUBLE_EQ(result.time_scale, 2.0);
  EXPECT_EQ(result.limiting_reason, "physical joint_1 deceleration");
  EXPECT_DOUBLE_EQ(motion.joint_trajectory.points[1].accelerations[0], -0.1);
}

TEST(TrajectorySpeedLimiter, RejectsInvalidOverrides)
{
  auto speed_settings = settings();
  speed_settings.joint_overrides_percent[4] = 0.0;
  auto motion = trajectory();
  EXPECT_THROW(control::apply_speed_limits(motion, nullptr, "", speed_settings, false), std::runtime_error);
}
