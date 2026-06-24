#include "robot_arm3_moveit_control/frame_transform_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_arm3_moveit_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z);
  if (orientation.norm() < 1e-12)
    throw std::runtime_error("Pose quaternion has zero length");
  orientation.normalize();

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = orientation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return result;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d& transform)
{
  const Eigen::Quaterniond orientation(transform.linear());
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

Eigen::Isometry3d frame_from_values(const std::array<double, 6>& values)
{
  for (const double value : values)
  {
    if (!std::isfinite(value))
      throw std::runtime_error("Frame parameters must be finite");
  }

  const double scale = kPi / 180.0;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(values[0], values[1], values[2]) / 1000.0;
  transform.linear() =
      (Eigen::AngleAxisd(values[3] * scale, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(values[4] * scale, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(values[5] * scale, Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  return transform;
}

}  // namespace

void FrameTransformManager::set_base(const std::array<double, 6>& values)
{
  world_from_base_ = frame_from_values(values);
  base_values_ = values;
  update_base_from_user();
}

void FrameTransformManager::set_user_frame(const std::array<double, 6>& values)
{
  world_from_user_ = frame_from_values(values);
  user_frame_values_ = values;
  update_base_from_user();
}

geometry_msgs::msg::Pose FrameTransformManager::command_to_base(
    const geometry_msgs::msg::Pose& user_pose) const
{
  return transform_pose(base_from_user_, user_pose);
}

geometry_msgs::msg::Pose FrameTransformManager::base_to_user(
    const geometry_msgs::msg::Pose& base_pose) const
{
  return transform_pose(base_from_user_.inverse(), base_pose);
}

const std::array<double, 6>& FrameTransformManager::base_values() const
{
  return base_values_;
}

const std::array<double, 6>& FrameTransformManager::user_frame_values() const
{
  return user_frame_values_;
}

const Eigen::Isometry3d& FrameTransformManager::base_from_user() const
{
  return base_from_user_;
}

void FrameTransformManager::update_base_from_user()
{
  base_from_user_ = world_from_base_.inverse() * world_from_user_;
}

geometry_msgs::msg::Pose transform_pose(const Eigen::Isometry3d& transform,
                                        const geometry_msgs::msg::Pose& pose)
{
  return eigen_to_pose(transform * pose_to_eigen(pose));
}

}  // namespace robot_arm3_moveit_control
