#include "robot_arm3_moveit_control/trajectory_speed_limiter.hpp"

#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace robot_arm3_moveit_control
{
namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;

double duration_seconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) / kNanosecondsPerSecond;
}

builtin_interfaces::msg::Duration seconds_to_duration(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0)
    throw std::runtime_error("Trajectory contains an invalid scaled time");

  const auto whole_seconds = static_cast<std::int32_t>(std::floor(seconds));
  auto nanoseconds = static_cast<std::uint32_t>(std::llround((seconds - whole_seconds) * kNanosecondsPerSecond));
  builtin_interfaces::msg::Duration duration;
  duration.sec = whole_seconds;
  if (nanoseconds >= 1000000000u)
  {
    duration.sec += 1;
    nanoseconds -= 1000000000u;
  }
  duration.nanosec = nanoseconds;
  return duration;
}

std::unordered_map<std::string, std::size_t> joint_indices(const std::vector<std::string>& names)
{
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t index = 0; index < names.size(); ++index)
    indices.emplace(names[index], index);

  for (const char* required : { "joint_1", "joint_2", "joint_7", "joint_4", "joint_5", "joint_6" })
  {
    if (indices.find(required) == indices.end())
      throw std::runtime_error(std::string("Trajectory is missing required joint ") + required);
  }
  return indices;
}

std::array<double, 6> physical_values(const std::vector<double>& values,
                                      const std::unordered_map<std::string, std::size_t>& indices)
{
  if (values.size() < indices.size())
    throw std::runtime_error("Trajectory point has an incomplete joint vector");
  const double joint_2 = values.at(indices.at("joint_2"));
  const double joint_7 = values.at(indices.at("joint_7"));
  return { values.at(indices.at("joint_1")), joint_2, joint_7 - joint_2, values.at(indices.at("joint_4")),
           values.at(indices.at("joint_5")), values.at(indices.at("joint_6")) };
}

void consider_ratio(double ratio, const std::string& reason, double& largest_ratio, std::string& limiting_reason)
{
  if (std::isfinite(ratio) && ratio > largest_ratio)
  {
    largest_ratio = ratio;
    limiting_reason = reason;
  }
}

bool same_positions(const trajectory_msgs::msg::JointTrajectoryPoint& left,
                    const trajectory_msgs::msg::JointTrajectoryPoint& right)
{
  if (left.positions.size() != right.positions.size())
    return false;
  for (std::size_t index = 0; index < left.positions.size(); ++index)
    if (std::abs(left.positions[index] - right.positions[index]) > 1e-12)
      return false;
  return true;
}

void collapse_zero_duration_duplicate_points(
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points)
{
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> normalized;
  normalized.reserve(points.size());
  for (auto& point : points)
  {
    if (normalized.empty())
    {
      normalized.push_back(std::move(point));
      continue;
    }

    const double delta_time =
        duration_seconds(point.time_from_start) -
        duration_seconds(normalized.back().time_from_start);
    if (std::abs(delta_time) <= 1e-12 &&
        same_positions(normalized.back(), point))
    {
      // Pilz may preserve the coincident end/start samples of adjacent
      // sequence items. Keep the latter sample's derivative fields while
      // removing the zero-duration duplicate.
      normalized.back() = std::move(point);
      continue;
    }
    normalized.push_back(std::move(point));
  }
  points = std::move(normalized);
}

}  // namespace

void validate_speed_settings(const MotionSpeedSettings& settings)
{
  for (std::size_t index = 0; index < 6; ++index)
  {
    if (!std::isfinite(settings.joint_overrides_percent[index]) || settings.joint_overrides_percent[index] < 1.0 ||
        settings.joint_overrides_percent[index] > 100.0)
      throw std::runtime_error("Joint override percentages must be within [1, 100]");
    if (!std::isfinite(settings.max_joint_velocities[index]) || settings.max_joint_velocities[index] <= 0.0)
      throw std::runtime_error("Physical joint maximum velocities must be positive");
    if (!std::isfinite(settings.max_joint_accelerations[index]) || settings.max_joint_accelerations[index] <= 0.0)
      throw std::runtime_error("Physical joint maximum accelerations must be positive");
  }
  if (!std::isfinite(settings.linear_speed_mps) || settings.linear_speed_mps <= 0.0)
    throw std::runtime_error("Linear speed must be positive");
  if (!std::isfinite(settings.acceleration_percent) || settings.acceleration_percent < 1.0 ||
      settings.acceleration_percent > 100.0 || !std::isfinite(settings.deceleration_percent) ||
      settings.deceleration_percent < 1.0 || settings.deceleration_percent > 100.0)
    throw std::runtime_error("Acceleration and deceleration percentages must be within [1, 100]");
}

ScalingResult apply_speed_limits(moveit_msgs::msg::RobotTrajectory& trajectory,
                                 const moveit::core::RobotModelConstPtr& robot_model,
                                 const std::string& end_effector_link, const MotionSpeedSettings& settings,
                                 bool enforce_linear_speed)
{
  validate_speed_settings(settings);
  auto& joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty())
    throw std::runtime_error("Cannot limit an empty trajectory");

  collapse_zero_duration_duplicate_points(joint_trajectory.points);

  const auto indices = joint_indices(joint_trajectory.joint_names);
  double largest_ratio = 1.0;
  std::string limiting_reason = "none";
  for (std::size_t point_index = 0; point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto& point = joint_trajectory.points[point_index];
    if (!point.velocities.empty())
    {
      const auto velocities = physical_values(point.velocities, indices);
      for (std::size_t axis = 0; axis < 6; ++axis)
      {
        const double allowed = settings.max_joint_velocities[axis] * settings.joint_overrides_percent[axis] / 100.0;
        consider_ratio(std::abs(velocities[axis]) / allowed,
                       "physical joint_" + std::to_string(axis + 1) + " velocity", largest_ratio, limiting_reason);
      }
    }

    if (!point.accelerations.empty())
    {
      const auto accelerations = physical_values(point.accelerations, indices);
      const bool has_velocities = !point.velocities.empty();
      const auto velocities = has_velocities ? physical_values(point.velocities, indices) : std::array<double, 6>{};
      for (std::size_t axis = 0; axis < 6; ++axis)
      {
        const bool decelerating = has_velocities && velocities[axis] * accelerations[axis] < -1e-12;
        const double phase_percent =
            decelerating ? settings.deceleration_percent : settings.acceleration_percent;
        const double allowed = settings.max_joint_accelerations[axis] * phase_percent / 100.0;
        consider_ratio(std::sqrt(std::abs(accelerations[axis]) / allowed),
                       "physical joint_" + std::to_string(axis + 1) +
                           (decelerating ? " deceleration" : " acceleration"),
                       largest_ratio, limiting_reason);
      }
    }

    if (point_index > 0)
    {
      const auto& previous = joint_trajectory.points[point_index - 1];
      const double delta_time = duration_seconds(point.time_from_start) - duration_seconds(previous.time_from_start);
      if (delta_time <= 0.0)
        throw std::runtime_error("Trajectory point times must be strictly increasing");
      const auto current_positions = physical_values(point.positions, indices);
      const auto previous_positions = physical_values(previous.positions, indices);
      for (std::size_t axis = 0; axis < 6; ++axis)
      {
        const double average_velocity = std::abs(current_positions[axis] - previous_positions[axis]) / delta_time;
        const double allowed = settings.max_joint_velocities[axis] * settings.joint_overrides_percent[axis] / 100.0;
        consider_ratio(average_velocity / allowed,
                       "physical joint_" + std::to_string(axis + 1) + " segment velocity", largest_ratio,
                       limiting_reason);
      }
    }
  }

  if (enforce_linear_speed)
  {
    if (!robot_model)
      throw std::runtime_error("Robot model is required to enforce TCP linear speed");
    if (!robot_model->hasLinkModel(end_effector_link))
      throw std::runtime_error("Unknown end-effector link for TCP speed limiting: " + end_effector_link);

    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();
    Eigen::Vector3d previous_position = Eigen::Vector3d::Zero();
    double previous_time = 0.0;
    bool have_previous = false;

    for (const auto& point : joint_trajectory.points)
    {
      if (point.positions.size() != joint_trajectory.joint_names.size())
        throw std::runtime_error("Trajectory point has an incomplete position vector");
      for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index)
        state.setVariablePosition(joint_trajectory.joint_names[index], point.positions[index]);
      state.update();
      const Eigen::Vector3d position = state.getGlobalLinkTransform(end_effector_link).translation();
      const double time = duration_seconds(point.time_from_start);
      if (have_previous)
      {
        const double delta_time = time - previous_time;
        if (delta_time <= 0.0)
          throw std::runtime_error("Trajectory point times must be strictly increasing");
        consider_ratio((position - previous_position).norm() / delta_time / settings.linear_speed_mps,
                       "TCP linear speed", largest_ratio, limiting_reason);
      }
      previous_position = position;
      previous_time = time;
      have_previous = true;
    }
  }

  if (largest_ratio > 1.0 + 1e-9)
  {
    for (auto& point : joint_trajectory.points)
    {
      point.time_from_start = seconds_to_duration(duration_seconds(point.time_from_start) * largest_ratio);
      for (double& velocity : point.velocities)
        velocity /= largest_ratio;
      for (double& acceleration : point.accelerations)
        acceleration /= largest_ratio * largest_ratio;
    }
  }

  return { largest_ratio, limiting_reason };
}

}  // namespace robot_arm3_moveit_control
