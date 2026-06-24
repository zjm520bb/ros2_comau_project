#include <gtest/gtest.h>

#include <Eigen/Core>

#include "robot_arm3_moveit_control/fly_queue.hpp"
#include "robot_arm3_moveit_control/pilz_sequence_builder.hpp"

namespace control = robot_arm3_moveit_control;

TEST(FlyQueue, EnforcesCapacityAndType)
{
  control::FlyQueue queue(2);
  queue.add(control::FlySegmentType::LINEAR, { 1, 2, 3, 4, 5, 6 });
  EXPECT_THROW(queue.add(control::FlySegmentType::JOINT, { 1, 2, 3, 4, 5, 6 }), control::FlyQueueError);
  queue.add(control::FlySegmentType::CIRCULAR, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });
  EXPECT_THROW(queue.add(control::FlySegmentType::LINEAR, { 1, 2, 3, 4, 5, 6 }), control::FlyQueueError);
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.max_points(), 2u);
  queue.clear();
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.type(), control::FlyQueueType::NONE);
}

TEST(FlyQueue, CartesianDistanceBecomesBlendRadiusAndLastRadiusIsZero)
{
  control::FlySettings settings;
  settings.mode = control::FlyMode::CARTESIAN;
  settings.distance_mm = 5.0;
  const std::vector<Eigen::Vector3d> points = { { 0, 0, 0 }, { 0.1, 0, 0 }, { 0.2, 0, 0 } };
  const auto radii = control::compute_fly_blend_radii(points, settings, 0.45);
  ASSERT_EQ(radii.size(), 2u);
  EXPECT_DOUBLE_EQ(radii[0], 0.005);
  EXPECT_DOUBLE_EQ(radii[1], 0.0);
}

TEST(FlyQueue, NormalPercentageUsesAdjacentDistances)
{
  control::FlySettings settings;
  settings.mode = control::FlyMode::NORMAL;
  settings.normal_percent = 50.0;
  const std::vector<Eigen::Vector3d> points = { { 0, 0, 0 }, { 0.1, 0, 0 }, { 0.1, 0.2, 0 } };
  const auto radii = control::compute_fly_blend_radii(points, settings, 0.45);
  ASSERT_EQ(radii.size(), 2u);
  EXPECT_NEAR(radii[0], 0.0225, 1e-12);
  EXPECT_DOUBLE_EQ(radii[1], 0.0);
}
