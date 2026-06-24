#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Geometry>

#include <array>

namespace robot_arm3_moveit_control
{

class FrameTransformManager
{
public:
  void set_base(const std::array<double, 6>& values);
  void set_user_frame(const std::array<double, 6>& values);

  geometry_msgs::msg::Pose command_to_base(const geometry_msgs::msg::Pose& user_pose) const;
  geometry_msgs::msg::Pose base_to_user(const geometry_msgs::msg::Pose& base_pose) const;

  const std::array<double, 6>& base_values() const;
  const std::array<double, 6>& user_frame_values() const;
  const Eigen::Isometry3d& base_from_user() const;

private:
  void update_base_from_user();

  std::array<double, 6> base_values_{};
  std::array<double, 6> user_frame_values_{};
  Eigen::Isometry3d world_from_base_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d world_from_user_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d base_from_user_{ Eigen::Isometry3d::Identity() };
};

geometry_msgs::msg::Pose transform_pose(const Eigen::Isometry3d& transform,
                                        const geometry_msgs::msg::Pose& pose);

}  // namespace robot_arm3_moveit_control
