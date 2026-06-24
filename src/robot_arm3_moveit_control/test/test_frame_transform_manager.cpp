#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "robot_arm3_moveit_control/frame_transform_manager.hpp"

namespace control = robot_arm3_moveit_control;

namespace
{

geometry_msgs::msg::Pose identity_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(FrameTransformManager, ConvertsUserCoordinatesThroughWorldAndBase)
{
  control::FrameTransformManager frames;
  frames.set_base({ 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  frames.set_user_frame({ 1000.0, 2000.0, 0.0, 0.0, 0.0, 0.0 });

  const auto in_base = frames.command_to_base(identity_pose(0.1, 0.2, 0.3));
  EXPECT_NEAR(in_base.position.x, 0.1, 1e-12);
  EXPECT_NEAR(in_base.position.y, 2.2, 1e-12);
  EXPECT_NEAR(in_base.position.z, 0.3, 1e-12);
}

TEST(FrameTransformManager, RoundTripsPositionAndOrientation)
{
  control::FrameTransformManager frames;
  frames.set_base({ 500.0, -200.0, 100.0, 20.0, 30.0, -10.0 });
  frames.set_user_frame({ -300.0, 400.0, 800.0, -15.0, 40.0, 25.0 });

  geometry_msgs::msg::Pose original = identity_pose(0.4, -0.6, 1.2);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY());
  original.orientation.x = rotation.x();
  original.orientation.y = rotation.y();
  original.orientation.z = rotation.z();
  original.orientation.w = rotation.w();

  const auto recovered = frames.base_to_user(frames.command_to_base(original));
  EXPECT_NEAR(recovered.position.x, original.position.x, 1e-12);
  EXPECT_NEAR(recovered.position.y, original.position.y, 1e-12);
  EXPECT_NEAR(recovered.position.z, original.position.z, 1e-12);
  const Eigen::Quaterniond expected(original.orientation.w, original.orientation.x, original.orientation.y,
                                    original.orientation.z);
  const Eigen::Quaterniond actual(recovered.orientation.w, recovered.orientation.x, recovered.orientation.y,
                                  recovered.orientation.z);
  EXPECT_NEAR(std::abs(expected.dot(actual)), 1.0, 1e-12);
}
