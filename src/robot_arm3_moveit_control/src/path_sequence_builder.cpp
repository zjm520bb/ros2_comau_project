#include "robot_arm3_moveit_control/path_sequence_builder.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>

namespace robot_arm3_moveit_control
{
namespace
{

using PathNode = arm_tcp_bridge_interfaces::msg::PathNode;
constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

Eigen::Isometry3d transform_from_values(const std::array<double, 6>& values, bool millimeters)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const double scale = millimeters ? 0.001 : 1.0;
  transform.translation() = Eigen::Vector3d(values[0] * scale, values[1] * scale, values[2] * scale);
  transform.linear() =
      (Eigen::AngleAxisd(radians(values[3]), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(radians(values[4]), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(radians(values[5]), Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose pose_from_transform(const Eigen::Isometry3d& transform)
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

std::map<std::uint8_t, Eigen::Isometry3d> frame_table(const ExecutePath::Goal& goal)
{
  std::map<std::uint8_t, Eigen::Isometry3d> frames;
  for (const auto& frame : goal.frames)
    frames.emplace(frame.index, transform_from_values(frame.pose, true));
  return frames;
}

geometry_msgs::msg::Pose node_pose(
    const PathNode& node, const std::map<std::uint8_t, Eigen::Isometry3d>& frames,
    const PathSequenceOptions& options)
{
  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity();
  if (node.reference_index != 0)
    reference = frames.at(node.reference_index);
  const Eigen::Isometry3d selected_target =
      options.base_from_user * reference * transform_from_values(node.target, true);
  if (node.tool_index == 0)
    return pose_from_transform(selected_target);

  const Eigen::Isometry3d flange_to_selected_tool = frames.at(node.tool_index);
  const Eigen::Isometry3d default_tool_target =
      selected_target * flange_to_selected_tool.inverse() * options.flange_to_default_tool;
  return pose_from_transform(default_tool_target);
}

std::array<double, 6> planning_joint_target(
    const PathNode& node, const PathSequenceOptions& options)
{
  const double joint_2 = radians(node.target[1]);
  const double joint_3 = radians(node.target[2]);
  const double joint_7 = joint_2 + joint_3 + options.coupling_offset;
  if (joint_3 < options.joint_3_min || joint_3 > options.joint_3_max)
    throw PathModelError("PATH joint target produces joint_3 outside its configured limits");
  if (joint_7 < options.joint_7_min || joint_7 > options.joint_7_max)
    throw PathModelError("PATH joint target produces joint_7 outside its configured limits");
  return { radians(node.target[0]), joint_2, joint_7, radians(node.target[3]),
           radians(node.target[4]), radians(node.target[5]) };
}

moveit_msgs::msg::Constraints joint_goal(const std::array<double, 6>& values)
{
  static const std::array<std::string, 6> names = {
    "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6"
  };
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

moveit_msgs::msg::Constraints cartesian_goal(
    const geometry_msgs::msg::Pose& pose, const PathSequenceOptions& options,
    double tolerance)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = options.base_frame;
  stamped.pose = pose;
  const double position_tolerance = tolerance > 0.0 ? tolerance / 1000.0 : 1e-4;
  return kinematic_constraints::constructGoalConstraints(
      options.end_effector_link, stamped, position_tolerance, 1e-3);
}

double blend_radius(
    const PathNode& node, bool last_item, double incoming_distance,
    double outgoing_distance, const PathSequenceOptions& options)
{
  if (last_item || !node.fly)
    return 0.0;
  if (node.fly_type == 1)
    return node.fly_distance_mm / 1000.0;
  return options.normal_radius_safety_factor * std::min(incoming_distance, outgoing_distance) *
         node.fly_percent / 100.0;
}

}  // namespace

moveit_msgs::msg::MotionSequenceRequest build_path_sequence_batch(
    const ExecutePath::Goal& goal, const std::vector<std::uint32_t>& node_indexes,
    const PathSequenceOptions& options, const moveit::core::RobotState& current_state,
    const moveit::core::RobotModelConstPtr& robot_model)
{
  if (node_indexes.empty())
    throw PathModelError("PATH batch is empty");
  if (!robot_model || !robot_model->hasLinkModel(options.end_effector_link))
    throw PathModelError("PATH robot model or end-effector link is invalid");
  const bool forward = node_indexes.size() < 2 || node_indexes.front() < node_indexes.back();
  if (!forward)
  {
    for (const auto index : node_indexes)
      if (goal.nodes.at(index - 1).motion_type == PathNode::CIRCULAR ||
          goal.nodes.at(index - 1).motion_type == PathNode::SEG_VIA)
        throw PathModelError("reverse CIRCULAR PATH is not supported by the simulation adapter");
  }

  const auto frames = frame_table(goal);
  std::vector<std::uint32_t> item_indexes;
  for (const auto index : node_indexes)
    if (goal.nodes.at(index - 1).motion_type != PathNode::SEG_VIA)
      item_indexes.push_back(index);
  if (item_indexes.empty())
    throw PathModelError("PATH batch contains no executable destination");

  moveit::core::RobotState updated_state(current_state);
  updated_state.update();
  std::vector<Eigen::Vector3d> points;
  points.push_back(updated_state.getGlobalLinkTransform(options.end_effector_link).translation());
  for (const auto index : item_indexes)
  {
    const auto& node = goal.nodes.at(index - 1);
    if (goal.path_type == ExecutePath::Goal::JOINT)
    {
      moveit::core::RobotState target_state(updated_state);
      const auto target = planning_joint_target(node, options);
      static const std::array<std::string, 6> names = {
        "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6"
      };
      for (std::size_t joint = 0; joint < names.size(); ++joint)
        target_state.setVariablePosition(names[joint], target[joint]);
      target_state.update();
      points.push_back(target_state.getGlobalLinkTransform(options.end_effector_link).translation());
    }
    else
    {
      const auto pose = node_pose(node, frames, options);
      points.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    }
  }

  moveit_msgs::msg::MotionSequenceRequest sequence;
  for (std::size_t item = 0; item < item_indexes.size(); ++item)
  {
    const std::uint32_t index = item_indexes[item];
    const auto& node = goal.nodes.at(index - 1);
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    auto& request = sequence_item.req;
    request.pipeline_id = options.pipeline_id;
    request.group_name = options.planning_group;
    request.allowed_planning_time = options.planning_time;
    request.num_planning_attempts = options.planning_attempts;
    const double segment_scale = std::clamp(node.segment_override / 100.0, 0.001, 1.0);
    if (goal.path_type == ExecutePath::Goal::JOINT)
      request.max_velocity_scaling_factor = segment_scale;
    else
    {
      const double speed = node.linear_speed > 0.0 ? node.linear_speed : options.default_linear_speed;
      request.max_velocity_scaling_factor =
          std::min(segment_scale, std::min(1.0, speed / options.pilz_max_trans_velocity));
    }
    request.max_acceleration_scaling_factor = segment_scale;
    if (item == 0)
      moveit::core::robotStateToRobotStateMsg(updated_state, request.start_state);

    if (goal.path_type == ExecutePath::Goal::JOINT)
    {
      request.planner_id = options.ptp_planner_id;
      request.goal_constraints.push_back(joint_goal(planning_joint_target(node, options)));
    }
    else
    {
      request.planner_id = node.motion_type == PathNode::JOINT
                               ? options.ptp_planner_id
                               : node.motion_type == PathNode::LINEAR ? options.lin_planner_id
                                                                    : options.circ_planner_id;
      request.goal_constraints.push_back(
          cartesian_goal(node_pose(node, frames, options), options, node.tolerance));
      if (node.motion_type == PathNode::CIRCULAR)
      {
        if (index < 2 || goal.nodes.at(index - 2).motion_type != PathNode::SEG_VIA)
          throw PathModelError("CIRCULAR PATH node has no SEG_VIA predecessor");
        moveit_msgs::msg::PositionConstraint interim;
        interim.header.frame_id = options.base_frame;
        interim.link_name = options.end_effector_link;
        interim.weight = 1.0;
        geometry_msgs::msg::Pose interim_pose;
        interim_pose.position = node_pose(goal.nodes.at(index - 2), frames, options).position;
        interim_pose.orientation.w = 1.0;
        interim.constraint_region.primitive_poses.push_back(interim_pose);
        request.path_constraints.name = "interim";
        request.path_constraints.position_constraints.push_back(interim);
      }
    }

    const double incoming = (points[item + 1] - points[item]).norm();
    const double outgoing =
        item + 2 < points.size() ? (points[item + 2] - points[item + 1]).norm() : incoming;
    sequence_item.blend_radius =
        blend_radius(node, item + 1 == item_indexes.size(), incoming, outgoing, options);
    sequence.items.push_back(std::move(sequence_item));
  }
  return sequence;
}

}  // namespace robot_arm3_moveit_control
