#include <arm_tcp_bridge_interfaces/action/execute_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "robot_arm3_moveit_control/command_parser.hpp"
#include "robot_arm3_moveit_control/coupled_trajectory_validator.hpp"
#include "robot_arm3_moveit_control/fly_queue.hpp"
#include "robot_arm3_moveit_control/pilz_sequence_builder.hpp"
#include "robot_arm3_moveit_control/trajectory_speed_limiter.hpp"
#include "robot_arm3_moveit_control/wrist_turn_preserver.hpp"

namespace robot_arm3_moveit_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

geometry_msgs::msg::Quaternion aer_to_quaternion(double a_degrees, double e_degrees, double r_degrees)
{
  const Eigen::Quaterniond quaternion =
      Eigen::AngleAxisd(degrees_to_radians(a_degrees), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(degrees_to_radians(e_degrees), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(degrees_to_radians(r_degrees), Eigen::Vector3d::UnitZ());

  const Eigen::Quaterniond normalized = quaternion.normalized();
  geometry_msgs::msg::Quaternion message;
  message.x = normalized.x();
  message.y = normalized.y();
  message.z = normalized.z();
  message.w = normalized.w();
  return message;
}

Eigen::Vector3d quaternion_to_aer(const geometry_msgs::msg::Quaternion& message)
{
  Eigen::Quaterniond quaternion(message.w, message.x, message.y, message.z);
  quaternion.normalize();
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  const double cosine_e = std::clamp(rotation(2, 2), -1.0, 1.0);
  const double e = std::acos(cosine_e);
  const double sine_e = std::sin(e);
  double a = 0.0;
  double r = 0.0;

  if (std::abs(sine_e) > 1e-9)
  {
    a = std::atan2(rotation(1, 2), rotation(0, 2));
    r = std::atan2(rotation(2, 1), -rotation(2, 0));
  }
  else if (cosine_e > 0.0)
  {
    a = std::atan2(rotation(1, 0), rotation(0, 0));
  }
  else
  {
    a = std::atan2(-rotation(1, 0), -rotation(0, 0));
  }

  return { radians_to_degrees(a), radians_to_degrees(e), radians_to_degrees(r) };
}

std::array<double, 6> six_values(const std::vector<double>& values, const std::string& parameter_name)
{
  if (values.size() != 6)
    throw std::runtime_error(parameter_name + " must contain exactly six values");
  return { values[0], values[1], values[2], values[3], values[4], values[5] };
}

double rounded_percent(double value, const std::string& name)
{
  if (!std::isfinite(value) || value < 1.0 || value > 100.0)
    throw std::runtime_error(name + " must be within [1, 100]");
  return std::round(value);
}

}  // namespace

class MoveItCommandServer : public rclcpp::Node
{
public:
  using ExecuteCommand = arm_tcp_bridge_interfaces::action::ExecuteCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCommand>;
  using SequenceAction = moveit_msgs::action::MoveGroupSequence;
  using SequenceGoalHandle = rclcpp_action::ClientGoalHandle<SequenceAction>;

  MoveItCommandServer() : Node("moveit_program_command_server")
  {
    declare_parameter<std::string>("action_name", "/sim/arm/execute");
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("end_effector_link", "Link_6");
    declare_parameter<std::string>("pilz_pipeline_id", "pilz_industrial_motion_planner");
    declare_parameter<std::string>("ptp_planner_id", "PTP");
    declare_parameter<std::string>("lin_planner_id", "LIN");
    declare_parameter<std::string>("circ_planner_id", "CIRC");
    declare_parameter<double>("pilz_max_trans_velocity", 1.0);
    declare_parameter<std::string>("sequence_action_name", "/sequence_move_group");
    declare_parameter<int>("max_fly_points", 20);
    declare_parameter<double>("fly_norm_radius_safety_factor", 0.45);
    declare_parameter<double>("sequence_wait_timeout", 10.0);
    declare_parameter<double>("planning_time", 5.0);
    declare_parameter<int>("planning_attempts", 10);
    declare_parameter<double>("current_state_timeout", 2.0);
    declare_parameter<std::vector<double>>("joint_overrides_percent", { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 });
    declare_parameter<std::vector<double>>("physical_joint_max_velocities", { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 });
    declare_parameter<std::vector<double>>("physical_joint_max_accelerations", { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 });
    declare_parameter<double>("linear_speed_mps", 0.05);
    declare_parameter<double>("acceleration_percent", 10.0);
    declare_parameter<double>("deceleration_percent", 10.0);
    declare_parameter<double>("coupling_offset", 1.5708);
    declare_parameter<double>("joint_3_min", -4.0317);
    declare_parameter<double>("joint_3_max", 0.0);
    declare_parameter<double>("joint_7_min", -1.151917306);
    declare_parameter<double>("joint_7_max", 1.047197551);
  }

  void initialize()
  {
    planning_group_ = get_parameter("planning_group").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    end_effector_link_ = get_parameter("end_effector_link").as_string();
    pilz_pipeline_id_ = get_parameter("pilz_pipeline_id").as_string();
    ptp_planner_id_ = get_parameter("ptp_planner_id").as_string();
    lin_planner_id_ = get_parameter("lin_planner_id").as_string();
    circ_planner_id_ = get_parameter("circ_planner_id").as_string();
    pilz_max_trans_velocity_ = get_parameter("pilz_max_trans_velocity").as_double();
    if (!std::isfinite(pilz_max_trans_velocity_) || pilz_max_trans_velocity_ <= 0.0)
      throw std::runtime_error("pilz_max_trans_velocity must be greater than zero");
    const int max_fly_points = get_parameter("max_fly_points").as_int();
    if (max_fly_points <= 0)
      throw std::runtime_error("max_fly_points must be greater than zero");
    fly_queue_ = std::make_unique<FlyQueue>(static_cast<std::size_t>(max_fly_points));
    fly_norm_radius_safety_factor_ = get_parameter("fly_norm_radius_safety_factor").as_double();
    if (!std::isfinite(fly_norm_radius_safety_factor_) || fly_norm_radius_safety_factor_ <= 0.0 ||
        fly_norm_radius_safety_factor_ >= 0.5)
      throw std::runtime_error("fly_norm_radius_safety_factor must be within (0, 0.5)");
    sequence_wait_timeout_ = get_parameter("sequence_wait_timeout").as_double();
    if (!std::isfinite(sequence_wait_timeout_) || sequence_wait_timeout_ <= 0.0)
      throw std::runtime_error("sequence_wait_timeout must be greater than zero");
    current_state_timeout_ = get_parameter("current_state_timeout").as_double();
    speed_settings_.joint_overrides_percent =
        six_values(get_parameter("joint_overrides_percent").as_double_array(), "joint_overrides_percent");
    speed_settings_.max_joint_velocities =
        six_values(get_parameter("physical_joint_max_velocities").as_double_array(), "physical_joint_max_velocities");
    speed_settings_.max_joint_accelerations = six_values(
        get_parameter("physical_joint_max_accelerations").as_double_array(), "physical_joint_max_accelerations");
    speed_settings_.linear_speed_mps = get_parameter("linear_speed_mps").as_double();
    speed_settings_.acceleration_percent = get_parameter("acceleration_percent").as_double();
    speed_settings_.deceleration_percent = get_parameter("deceleration_percent").as_double();
    validate_speed_settings(speed_settings_);
    coupling_offset_ = get_parameter("coupling_offset").as_double();
    joint_3_min_ = get_parameter("joint_3_min").as_double();
    joint_3_max_ = get_parameter("joint_3_max").as_double();
    joint_7_min_ = get_parameter("joint_7_min").as_double();
    joint_7_max_ = get_parameter("joint_7_max").as_double();

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    if (!move_group_->setEndEffectorLink(end_effector_link_))
      throw std::runtime_error("Unknown end-effector link: " + end_effector_link_);
    planning_time_ = get_parameter("planning_time").as_double();
    planning_attempts_ = get_parameter("planning_attempts").as_int();
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    // Plan at the configured joint limits, then apply the C4G-style physical
    // overrides to the complete synchronized trajectory before execution.
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    sequence_client_ = rclcpp_action::create_client<SequenceAction>(
        this, get_parameter("sequence_action_name").as_string());

    const std::string action_name = get_parameter("action_name").as_string();
    action_server_ = rclcpp_action::create_server<ExecuteCommand>(
        this, action_name,
        std::bind(&MoveItCommandServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveItCommandServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveItCommandServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Programmatic MoveIt server ready on %s", action_name.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const ExecuteCommand::Goal> goal)
  {
    if (busy_.load())
    {
      RCLCPP_WARN(get_logger(), "Rejecting command because the server is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }

    try
    {
      (void)parse_command(goal->command);
    }
    catch (const CommandParseError& error)
    {
      RCLCPP_WARN(get_logger(), "Rejecting command: %s", error.what());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      if (active_sequence_goal_ && sequence_client_)
        sequence_client_->async_cancel_goal(active_sequence_goal_);
    }
    if (move_group_)
      move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (busy_.exchange(true))
    {
      auto result = std::make_shared<ExecuteCommand::Result>();
      result->success = false;
      result->message = "Server became busy before execution";
      goal_handle->abort(result);
      return;
    }
    std::thread(&MoveItCommandServer::execute, this, goal_handle).detach();
  }

  void publish_feedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& state)
  {
    auto feedback = std::make_shared<ExecuteCommand::Feedback>();
    feedback->state = state;
    goal_handle->publish_feedback(feedback);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<ExecuteCommand::Result>();
    try
    {
      const ParsedCommand command = parse_command(goal_handle->get_goal()->command);
      result->message = dispatch(command, goal_handle);

      if (goal_handle->is_canceling())
      {
        result->success = false;
        result->message = "Command canceled";
        goal_handle->canceled(result);
      }
      else
      {
        result->success = true;
        goal_handle->succeed(result);
      }
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(get_logger(), "Command failed: %s", error.what());
      result->success = false;
      result->message = error.what();
      if (goal_handle->is_canceling())
        goal_handle->canceled(result);
      else
        goal_handle->abort(result);
    }
    busy_.store(false);
  }

  std::string dispatch(const ParsedCommand& command, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    switch (command.type)
    {
      case CommandType::HELLO:
        return "Hello from the MoveIt simulation server";
      case CommandType::GET_POSE:
        return get_pose();
      case CommandType::GET_JOINTS:
        publish_feedback(goal_handle, "reading_joint_state");
        return get_joints();
      case CommandType::SET_ORIENTATION:
        return set_orientation(command.values);
      case CommandType::SET_SPEED_JOINT:
        return set_speed_joint(command.values);
      case CommandType::SET_JOINT_OVERRIDES:
        return set_joint_overrides(command.values);
      case CommandType::SET_SPEED_LINEAR:
        return set_speed_linear(command.values);
      case CommandType::SET_ACCELERATION:
        return set_acceleration(command.values);
      case CommandType::SET_DECELERATION:
        return set_deceleration(command.values);
      case CommandType::MOVE_JOINT:
        return move_joint(command.values, goal_handle);
      case CommandType::MOVE_LINEAR:
        return move_linear(command.values, goal_handle);
      case CommandType::MOVE_CIRCULAR:
        return move_circular(command.values, goal_handle);
      case CommandType::SET_FLY_CART:
        return set_fly_cart(command.values);
      case CommandType::SET_FLY_NORM:
        return set_fly_norm(command.values);
      case CommandType::CLEAR_FLY_QUEUE:
        return clear_fly_queue();
      case CommandType::ADD_FLY_LINEAR:
        return add_fly_segment(FlySegmentType::LINEAR, command.values);
      case CommandType::ADD_FLY_CIRCULAR:
        return add_fly_segment(FlySegmentType::CIRCULAR, command.values);
      case CommandType::ADD_FLY_JOINT:
        return add_fly_segment(FlySegmentType::JOINT, command.values);
      case CommandType::EXECUTE_FLY_QUEUE:
        return execute_fly_queue(goal_handle);
    }
    throw std::runtime_error("Unhandled command type");
  }

  std::string set_orientation(const std::vector<double>& values)
  {
    const double mode = values.at(0);
    if (std::abs(mode) > 1e-9)
      throw std::runtime_error("Only setOrientation:0 (RS_WORLD) is supported in this version");
    return "Orientation mode set to RS_WORLD";
  }

  std::string set_speed_joint(const std::vector<double>& values)
  {
    const double percent = rounded_percent(values.at(0), "Joint speed override");
    speed_settings_.joint_overrides_percent.fill(percent);
    return "Joint speed override set to " + std::to_string(static_cast<int>(percent)) + "% for all six axes";
  }

  std::string set_joint_overrides(const std::vector<double>& values)
  {
    for (std::size_t axis = 0; axis < 6; ++axis)
      speed_settings_.joint_overrides_percent[axis] =
          rounded_percent(values.at(axis), "Joint override for axis " + std::to_string(axis + 1));

    std::ostringstream message;
    message << "Individual joint overrides set to [";
    for (std::size_t axis = 0; axis < 6; ++axis)
    {
      if (axis > 0)
        message << ",";
      message << static_cast<int>(speed_settings_.joint_overrides_percent[axis]);
    }
    message << "]%";
    return message.str();
  }

  std::string set_speed_linear(const std::vector<double>& values)
  {
    const double speed = values.at(0);
    if (!std::isfinite(speed) || speed <= 0.0)
      throw std::runtime_error("Linear speed must be greater than zero");
    speed_settings_.linear_speed_mps = speed;
    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "TCP linear speed set to " << speed << " m/s";
    return message.str();
  }

  std::string set_acceleration(const std::vector<double>& values)
  {
    speed_settings_.acceleration_percent = rounded_percent(values.at(0), "Acceleration override");
    return "Acceleration override set to " +
           std::to_string(static_cast<int>(speed_settings_.acceleration_percent)) + "%";
  }

  std::string set_deceleration(const std::vector<double>& values)
  {
    speed_settings_.deceleration_percent = rounded_percent(values.at(0), "Deceleration override");
    return "Deceleration override set to " +
           std::to_string(static_cast<int>(speed_settings_.deceleration_percent)) + "%";
  }

  std::string set_fly_cart(const std::vector<double>& values)
  {
    const double stress = rounded_percent(values.at(0), "FLY stress percentage");
    const double raw_mode = values.at(1);
    const int trajectory_mode = static_cast<int>(std::round(raw_mode));
    if (std::abs(raw_mode - trajectory_mode) > 1e-9 || trajectory_mode < 0 || trajectory_mode > 3)
      throw std::runtime_error("FLY trajectory mode must be an integer within [0, 3]");
    const double distance = values.at(2);
    if (!std::isfinite(distance) || distance < 0.0)
      throw std::runtime_error("FLY distance must be non-negative");
    fly_settings_.mode = FlyMode::CARTESIAN;
    fly_settings_.stress_percent = stress;
    fly_settings_.trajectory_mode = trajectory_mode;
    fly_settings_.distance_mm = distance;
    std::ostringstream message;
    message << "FLY_CART configured: stress=" << static_cast<int>(stress) << "%, mode=" << trajectory_mode
            << ", Pilz blend_radius=" << std::fixed << std::setprecision(3) << distance << " mm";
    return message.str();
  }

  std::string set_fly_norm(const std::vector<double>& values)
  {
    fly_settings_.mode = FlyMode::NORMAL;
    fly_settings_.normal_percent = rounded_percent(values.at(0), "FLY normal percentage");
    return "FLY_NORM configured: " + std::to_string(static_cast<int>(fly_settings_.normal_percent)) + "%";
  }

  std::string clear_fly_queue()
  {
    const std::size_t previous_size = fly_queue_->size();
    fly_queue_->clear();
    return "FLY queue cleared; removed " + std::to_string(previous_size) + " element(s)";
  }

  std::string add_fly_segment(FlySegmentType type, const std::vector<double>& values)
  {
    fly_queue_->add(type, values);
    return "FLY segment added; queue_count=" + std::to_string(fly_queue_->size());
  }

  std::string scaling_description(const ScalingResult& result) const
  {
    std::ostringstream message;
    message << std::fixed << std::setprecision(3) << "time_scale=" << result.time_scale
            << ", limiting=" << result.limiting_reason;
    return message.str();
  }

  void select_pilz_planner(const std::string& planner_id, bool cartesian_motion)
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setPlanningPipelineId(pilz_pipeline_id_);
    move_group_->setPlannerId(planner_id);
    const double velocity_scaling = cartesian_motion ?
        std::min(1.0, speed_settings_.linear_speed_mps / pilz_max_trans_velocity_) :
        *std::max_element(speed_settings_.joint_overrides_percent.begin(),
                          speed_settings_.joint_overrides_percent.end()) / 100.0;
    const double acceleration_scaling =
        std::min(speed_settings_.acceleration_percent, speed_settings_.deceleration_percent) / 100.0;
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    RCLCPP_INFO(get_logger(), "Pilz %s request scaling: velocity=%.3f, acceleration=%.3f", planner_id.c_str(),
                velocity_scaling, acceleration_scaling);
  }

  geometry_msgs::msg::Pose cartesian_pose(const std::vector<double>& values, std::size_t offset) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = values.at(offset) / 1000.0;
    pose.position.y = values.at(offset + 1) / 1000.0;
    pose.position.z = values.at(offset + 2) / 1000.0;
    pose.orientation = aer_to_quaternion(values.at(offset + 3), values.at(offset + 4), values.at(offset + 5));
    return pose;
  }

  ScalingResult postprocess_trajectory(moveit_msgs::msg::RobotTrajectory& trajectory, bool preserve_wrist_turns,
                                       bool enforce_linear_speed)
  {
    if (preserve_wrist_turns)
    {
      const moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(current_state_timeout_);
      if (!current_state)
        throw std::runtime_error("Unable to read current wrist turns");
      const auto robot_model = move_group_->getRobotModel();
      for (const std::string& joint_name : { std::string("joint_4"), std::string("joint_6") })
      {
        const auto& bounds = robot_model->getVariableBounds(joint_name);
        preserve_absolute_turns(trajectory, joint_name, current_state->getVariablePosition(joint_name),
                                bounds.min_position_, bounds.max_position_);
      }
    }

    validate_coupled_trajectory(trajectory, coupling_offset_, joint_3_min_, joint_3_max_, joint_7_min_, joint_7_max_);
    return apply_speed_limits(trajectory, move_group_->getRobotModel(), end_effector_link_, speed_settings_,
                              enforce_linear_speed);
  }

  std::string move_joint(const std::vector<double>& degrees, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_joint_target");
    const double joint_1 = degrees_to_radians(degrees.at(0));
    const double joint_2 = degrees_to_radians(degrees.at(1));
    const double joint_3 = degrees_to_radians(degrees.at(2));
    const double joint_4 = degrees_to_radians(degrees.at(3));
    const double joint_5 = degrees_to_radians(degrees.at(4));
    const double joint_6 = degrees_to_radians(degrees.at(5));
    const double joint_7 = joint_2 + joint_3 + coupling_offset_;

    if (joint_3 < joint_3_min_ || joint_3 > joint_3_max_)
      throw std::runtime_error("moveJoint target produces joint_3 outside its configured limits");
    if (joint_7 < joint_7_min_ || joint_7 > joint_7_max_)
      throw std::runtime_error("moveJoint target produces joint_7 outside [-66, 60] degrees");

    select_pilz_planner(ptp_planner_id_, false);
    move_group_->setStartStateToCurrentState();
    const std::map<std::string, double> target = { { "joint_1", joint_1 }, { "joint_2", joint_2 },
                                                   { "joint_7", joint_7 }, { "joint_4", joint_4 },
                                                   { "joint_5", joint_5 }, { "joint_6", joint_6 } };
    if (!move_group_->setJointValueTarget(target))
      throw std::runtime_error("MoveIt rejected the requested joint target");

    publish_feedback(goal_handle, "planning_pilz_ptp");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to plan the joint motion");
    publish_feedback(goal_handle, "validating_and_retiming_pilz_ptp");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, false, false);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";

    publish_feedback(goal_handle, "executing_joint_motion");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the joint trajectory");
    return "Joint motion finished; " + scaling_description(scaling);
  }

  std::string move_linear(const std::vector<double>& values, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_linear_target");
    const geometry_msgs::msg::Pose target = cartesian_pose(values, 0);
    select_pilz_planner(lin_planner_id_, true);
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target, end_effector_link_);
    publish_feedback(goal_handle, "planning_pilz_lin");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Pilz failed to plan the LIN motion");

    publish_feedback(goal_handle, "validating_and_retiming_pilz_lin");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, true, true);

    if (goal_handle->is_canceling())
      return "Command canceled before execution";
    publish_feedback(goal_handle, "executing_pilz_lin");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the Pilz LIN trajectory");
    return "Linear motion finished; " + scaling_description(scaling);
  }

  std::string move_circular(const std::vector<double>& values, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_circular_target");
    const geometry_msgs::msg::Pose interim = cartesian_pose(values, 0);
    const geometry_msgs::msg::Pose target = cartesian_pose(values, 6);

    select_pilz_planner(circ_planner_id_, true);
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target, end_effector_link_);

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "interim";
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = base_frame_;
    position_constraint.link_name = end_effector_link_;
    position_constraint.weight = 1.0;
    geometry_msgs::msg::Pose interim_point_pose;
    interim_point_pose.position = interim.position;
    interim_point_pose.orientation.w = 1.0;
    // Pilz uses this pose as CIRC metadata. Adding a primitive would make
    // MoveIt enforce the interim point as a constraint on every path state.
    position_constraint.constraint_region.primitive_poses.push_back(interim_point_pose);
    path_constraints.position_constraints.push_back(position_constraint);
    move_group_->setPathConstraints(path_constraints);

    publish_feedback(goal_handle, "planning_pilz_circ");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Pilz failed to plan the CIRC motion");

    publish_feedback(goal_handle, "validating_and_retiming_pilz_circ");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, true, true);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";
    publish_feedback(goal_handle, "executing_pilz_circ");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the Pilz CIRC trajectory");
    return "Circular motion finished; " + scaling_description(scaling);
  }

  std::string execute_fly_queue(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    if (fly_queue_->size() < 2)
      throw std::runtime_error("At least two FLY queue elements are required");
    if (!sequence_client_->wait_for_action_server(std::chrono::duration<double>(sequence_wait_timeout_)))
      throw std::runtime_error("Pilz sequence action server is not available");

    const moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(current_state_timeout_);
    if (!current_state)
      throw std::runtime_error("Unable to read current state for FLY sequence");
    current_state->update();

    SequenceBuildOptions options;
    options.planning_group = planning_group_;
    options.base_frame = base_frame_;
    options.end_effector_link = end_effector_link_;
    options.pipeline_id = pilz_pipeline_id_;
    options.ptp_planner_id = ptp_planner_id_;
    options.lin_planner_id = lin_planner_id_;
    options.circ_planner_id = circ_planner_id_;
    options.planning_time = planning_time_;
    options.planning_attempts = planning_attempts_;
    options.pilz_max_trans_velocity = pilz_max_trans_velocity_;
    options.coupling_offset = coupling_offset_;
    options.joint_3_min = joint_3_min_;
    options.joint_3_max = joint_3_max_;
    options.joint_7_min = joint_7_min_;
    options.joint_7_max = joint_7_max_;
    options.normal_radius_safety_factor = fly_norm_radius_safety_factor_;

    publish_feedback(goal_handle, "building_fly_sequence");
    SequenceAction::Goal sequence_goal;
    sequence_goal.request = build_pilz_sequence(*fly_queue_, fly_settings_, speed_settings_, options, *current_state,
                                                move_group_->getRobotModel());
    sequence_goal.planning_options.plan_only = true;

    publish_feedback(goal_handle, "planning_pilz_sequence");
    rclcpp_action::Client<SequenceAction>::SendGoalOptions send_options;
    send_options.feedback_callback = [this, goal_handle](SequenceGoalHandle::SharedPtr,
                                                         const std::shared_ptr<const SequenceAction::Feedback> feedback) {
      publish_feedback(goal_handle, "pilz_sequence_" + feedback->state);
    };
    auto goal_future = sequence_client_->async_send_goal(sequence_goal, send_options);
    if (goal_future.wait_for(std::chrono::duration<double>(sequence_wait_timeout_)) != std::future_status::ready)
      throw std::runtime_error("Timed out while sending Pilz sequence goal");
    const SequenceGoalHandle::SharedPtr sequence_goal_handle = goal_future.get();
    if (!sequence_goal_handle)
      throw std::runtime_error("Pilz rejected the sequence planning request");
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      active_sequence_goal_ = sequence_goal_handle;
    }

    auto result_future = sequence_client_->async_get_result(sequence_goal_handle);
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
      if (goal_handle->is_canceling())
      {
        sequence_client_->async_cancel_goal(sequence_goal_handle);
        {
          std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
          active_sequence_goal_.reset();
        }
        throw std::runtime_error("FLY sequence planning canceled");
      }
    }
    const auto wrapped_result = result_future.get();
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      active_sequence_goal_.reset();
    }
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result)
      throw std::runtime_error("Pilz sequence planning action failed");

    const auto& response = wrapped_result.result->response;
    if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      throw std::runtime_error("Pilz sequence planner returned error code " +
                               std::to_string(response.error_code.val));
    if (response.planned_trajectories.size() != 1)
      throw std::runtime_error("Expected one combined arm trajectory from Pilz sequence planner");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.start_state_ = response.sequence_start;
    plan.trajectory_ = response.planned_trajectories.front();
    plan.planning_time_ = response.planning_time;
    const bool cartesian_queue = fly_queue_->type() == FlyQueueType::CARTESIAN;
    publish_feedback(goal_handle, "validating_and_retiming_fly_sequence");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, cartesian_queue, cartesian_queue);

    if (goal_handle->is_canceling())
      return "Command canceled before FLY sequence execution";
    publish_feedback(goal_handle, "executing_fly_sequence");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the FLY sequence");

    const std::size_t executed_count = fly_queue_->size();
    fly_queue_->clear();
    return "FLY sequence finished; segments=" + std::to_string(executed_count) + "; " +
           scaling_description(scaling);
  }

  std::string get_pose()
  {
    const geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose(end_effector_link_);
    const Eigen::Vector3d aer = quaternion_to_aer(pose.pose.orientation);
    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "frame=" << pose.header.frame_id << ", X="
            << pose.pose.position.x * 1000.0 << ", Y=" << pose.pose.position.y * 1000.0 << ", Z="
            << pose.pose.position.z * 1000.0 << ", A=" << aer.x() << ", E=" << aer.y() << ", R=" << aer.z();
    return message.str();
  }

  std::string get_joints()
  {
    const moveit::core::RobotStatePtr state = move_group_->getCurrentState(current_state_timeout_);
    if (!state)
      throw std::runtime_error("Unable to read the current robot joint state");

    const double joint_1 = state->getVariablePosition("joint_1");
    const double joint_2 = state->getVariablePosition("joint_2");
    const double joint_7 = state->getVariablePosition("joint_7");
    const double joint_4 = state->getVariablePosition("joint_4");
    const double joint_5 = state->getVariablePosition("joint_5");
    const double joint_6 = state->getVariablePosition("joint_6");
    const double calculated_joint_3 = joint_7 - joint_2 - coupling_offset_;
    const double calculated_joint_8 = -joint_7;
    const double reported_joint_3 = state->getVariablePosition("joint_3");
    const double reported_joint_8 = state->getVariablePosition("joint_8");

    std::ostringstream message;
    message << std::fixed << std::setprecision(6)
            << "unit=deg; moveit=[joint_1=" << radians_to_degrees(joint_1)
            << ", joint_2=" << radians_to_degrees(joint_2)
            << ", joint_7=" << radians_to_degrees(joint_7)
            << ", joint_4=" << radians_to_degrees(joint_4)
            << ", joint_5=" << radians_to_degrees(joint_5)
            << ", joint_6=" << radians_to_degrees(joint_6)
            << "]; coupled=[joint_3=" << radians_to_degrees(calculated_joint_3)
            << ", joint_8=" << radians_to_degrees(calculated_joint_8)
            << "]; reported_passive=[joint_3=" << radians_to_degrees(reported_joint_3)
            << ", joint_8=" << radians_to_degrees(reported_joint_8)
            << "]; coupling_error=[joint_3=" << radians_to_degrees(reported_joint_3 - calculated_joint_3)
            << ", joint_8=" << radians_to_degrees(reported_joint_8 - calculated_joint_8) << "]";
    return message.str();
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<ExecuteCommand>::SharedPtr action_server_;
  rclcpp_action::Client<SequenceAction>::SharedPtr sequence_client_;
  SequenceGoalHandle::SharedPtr active_sequence_goal_;
  std::mutex sequence_goal_mutex_;
  std::atomic_bool busy_{ false };
  std::string planning_group_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string pilz_pipeline_id_;
  std::string ptp_planner_id_;
  std::string lin_planner_id_;
  std::string circ_planner_id_;
  double current_state_timeout_{ 2.0 };
  MotionSpeedSettings speed_settings_;
  std::unique_ptr<FlyQueue> fly_queue_;
  FlySettings fly_settings_;
  double pilz_max_trans_velocity_{ 1.0 };
  double fly_norm_radius_safety_factor_{ 0.45 };
  double sequence_wait_timeout_{ 10.0 };
  double planning_time_{ 5.0 };
  int planning_attempts_{ 1 };
  double coupling_offset_{ 1.5708 };
  double joint_3_min_{ -4.0317 };
  double joint_3_max_{ 0.0 };
  double joint_7_min_{ -1.151917306 };
  double joint_7_max_{ 1.047197551 };
};

}  // namespace robot_arm3_moveit_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_arm3_moveit_control::MoveItCommandServer>();
  try
  {
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to start MoveIt command server: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
