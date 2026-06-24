#include "robot_arm3_moveit_control/pilz_sequence_builder.hpp"

#include "robot_arm3_moveit_control/frame_transform_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace robot_arm3_moveit_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Pose pose_from_values(const std::vector<double>& values, std::size_t offset)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = values.at(offset) / 1000.0;
  pose.position.y = values.at(offset + 1) / 1000.0;
  pose.position.z = values.at(offset + 2) / 1000.0;
  const Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(radians(values.at(offset + 3)), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(radians(values.at(offset + 4)), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(radians(values.at(offset + 5)), Eigen::Vector3d::UnitZ());
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

geometry_msgs::msg::Pose command_pose_from_values(const std::vector<double>& values, std::size_t offset,
                                                  const SequenceBuildOptions& options)
{
  return transform_pose(options.base_from_user, pose_from_values(values, offset));
}

std::array<double, 6> planning_joint_target(const std::vector<double>& values, const SequenceBuildOptions& options)
{
  const double joint_2 = radians(values.at(1));
  const double joint_3 = radians(values.at(2));
  const double joint_7 = joint_2 + joint_3 + options.coupling_offset;
  if (joint_3 < options.joint_3_min || joint_3 > options.joint_3_max)
    throw std::runtime_error("FLY joint target produces joint_3 outside its configured limits");
  if (joint_7 < options.joint_7_min || joint_7 > options.joint_7_max)
    throw std::runtime_error("FLY joint target produces joint_7 outside its configured limits");
  return { radians(values.at(0)), joint_2, joint_7, radians(values.at(3)), radians(values.at(4)),
           radians(values.at(5)) };
}

moveit_msgs::msg::Constraints joint_goal(const std::array<double, 6>& values)
{
  static const std::array<std::string, 6> names = { "joint_1", "joint_2", "joint_7",
                                                    "joint_4", "joint_5", "joint_6" };
  moveit_msgs::msg::Constraints constraints;
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = names[index];
    constraint.position = values[index];
    constraint.tolerance_above = 1e-4;
    constraint.tolerance_below = 1e-4;
    constraint.weight = 1.0;
    constraints.joint_constraints.push_back(constraint);
  }
  return constraints;
}

moveit_msgs::msg::Constraints cartesian_goal(const geometry_msgs::msg::Pose& pose,
                                             const SequenceBuildOptions& options)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = options.base_frame;
  stamped.pose = pose;
  return kinematic_constraints::constructGoalConstraints(options.end_effector_link, stamped, 1e-4, 1e-3);
}

double velocity_scaling(FlySegmentType type, const MotionSpeedSettings& settings,
                        const SequenceBuildOptions& options)
{
  if (type == FlySegmentType::JOINT)
    return *std::max_element(settings.joint_overrides_percent.begin(), settings.joint_overrides_percent.end()) /
           100.0;
  return std::min(1.0, settings.linear_speed_mps / options.pilz_max_trans_velocity);
}

}  // namespace

std::vector<double> compute_fly_blend_radii(const std::vector<Eigen::Vector3d>& points,
                                            const FlySettings& settings, double normal_radius_safety_factor)
{
  if (points.size() < 3)
    throw std::runtime_error("At least a start point and two FLY targets are required");
  std::vector<double> radii(points.size() - 1, 0.0);
  for (std::size_t target_index = 1; target_index + 1 < points.size(); ++target_index)
  {
    if (settings.mode == FlyMode::CARTESIAN)
      radii[target_index - 1] = settings.distance_mm / 1000.0;
    else
    {
      const double incoming = (points[target_index] - points[target_index - 1]).norm();
      const double outgoing = (points[target_index + 1] - points[target_index]).norm();
      radii[target_index - 1] = normal_radius_safety_factor * std::min(incoming, outgoing) *
                                settings.normal_percent / 100.0;
    }
  }
  return radii;
}

moveit_msgs::msg::MotionSequenceRequest build_pilz_sequence(
    const FlyQueue& queue, const FlySettings& fly_settings, const MotionSpeedSettings& speed_settings,
    const SequenceBuildOptions& options, const moveit::core::RobotState& current_state,
    const moveit::core::RobotModelConstPtr& robot_model)
{
  if (queue.size() < 2)
    throw std::runtime_error("At least two FLY queue elements are required");
  if (!robot_model || !robot_model->hasLinkModel(options.end_effector_link))
    throw std::runtime_error("Robot model or FLY end-effector link is invalid");
  if (queue.type() == FlyQueueType::CARTESIAN && fly_settings.mode != FlyMode::CARTESIAN)
    throw std::runtime_error("CARTESIAN FLY queue requires setFlyCart");
  if (queue.type() == FlyQueueType::JOINT && fly_settings.mode != FlyMode::NORMAL)
    throw std::runtime_error("JOINT FLY queue requires setFlyNorm");

  moveit::core::RobotState updated_current_state(current_state);
  updated_current_state.update();

  std::vector<Eigen::Vector3d> target_points;
  target_points.push_back(updated_current_state.getGlobalLinkTransform(options.end_effector_link).translation());
  for (const FlySegment& segment : queue.segments())
  {
    if (segment.type == FlySegmentType::JOINT)
    {
      moveit::core::RobotState target_state(updated_current_state);
      const auto target = planning_joint_target(segment.values, options);
      static const std::array<std::string, 6> names = { "joint_1", "joint_2", "joint_7",
                                                        "joint_4", "joint_5", "joint_6" };
      for (std::size_t index = 0; index < names.size(); ++index)
        target_state.setVariablePosition(names[index], target[index]);
      target_state.update();
      target_points.push_back(target_state.getGlobalLinkTransform(options.end_effector_link).translation());
    }
    else
    {
      const auto pose = command_pose_from_values(
          segment.values, segment.type == FlySegmentType::CIRCULAR ? 6 : 0, options);
      target_points.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    }
  }

  const std::vector<double> radii =
      compute_fly_blend_radii(target_points, fly_settings, options.normal_radius_safety_factor);
  const double acceleration_scaling =
      std::min(speed_settings.acceleration_percent, speed_settings.deceleration_percent) / 100.0;

  moveit_msgs::msg::MotionSequenceRequest sequence;
  for (std::size_t index = 0; index < queue.segments().size(); ++index)
  {
    const FlySegment& segment = queue.segments()[index];
    moveit_msgs::msg::MotionSequenceItem item;
    auto& request = item.req;
    request.pipeline_id = options.pipeline_id;
    request.group_name = options.planning_group;
    request.allowed_planning_time = options.planning_time;
    request.num_planning_attempts = options.planning_attempts;
    request.max_velocity_scaling_factor = velocity_scaling(segment.type, speed_settings, options);
    request.max_acceleration_scaling_factor = acceleration_scaling;
    if (index == 0)
      moveit::core::robotStateToRobotStateMsg(updated_current_state, request.start_state);

    if (segment.type == FlySegmentType::JOINT)
    {
      request.planner_id = options.ptp_planner_id;
      request.goal_constraints.push_back(joint_goal(planning_joint_target(segment.values, options)));
    }
    else
    {
      request.planner_id =
          segment.type == FlySegmentType::LINEAR ? options.lin_planner_id : options.circ_planner_id;
      request.goal_constraints.push_back(
          cartesian_goal(command_pose_from_values(
              segment.values, segment.type == FlySegmentType::CIRCULAR ? 6 : 0, options), options));
      if (segment.type == FlySegmentType::CIRCULAR)
      {
        moveit_msgs::msg::PositionConstraint interim;
        interim.header.frame_id = options.base_frame;
        interim.link_name = options.end_effector_link;
        interim.weight = 1.0;
        geometry_msgs::msg::Pose interim_pose;
        interim_pose.position = command_pose_from_values(segment.values, 0, options).position;
        interim_pose.orientation.w = 1.0;
        interim.constraint_region.primitive_poses.push_back(interim_pose);
        request.path_constraints.name = "interim";
        request.path_constraints.position_constraints.push_back(interim);
      }
    }
    item.blend_radius = radii[index];
    sequence.items.push_back(std::move(item));
  }
  return sequence;
}

}  // namespace robot_arm3_moveit_control
