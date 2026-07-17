#include <arm_tcp_bridge_interfaces/action/execute_command.hpp>
#include <arm_tcp_bridge_interfaces/action/execute_path.hpp>
#include <arm_tcp_bridge_interfaces/msg/path_block.hpp>
#include <arm_tcp_bridge_interfaces/msg/path_event.hpp>
#include <arm_tcp_bridge_interfaces/msg/path_frame.hpp>
#include <arm_tcp_bridge_interfaces/srv/get_path_state.hpp>
#include <arm_tcp_bridge_interfaces/srv/signal_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
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
#include "robot_arm3_moveit_control/frame_transform_manager.hpp"
#include "robot_arm3_moveit_control/pilz_sequence_builder.hpp"
#include "robot_arm3_moveit_control/path_model.hpp"
#include "robot_arm3_moveit_control/path_sequence_builder.hpp"
#include "robot_arm3_moveit_control/ompl_trajectory_to_path.hpp"
#include "robot_arm3_moveit_control/trajectory_speed_limiter.hpp"
#include "robot_arm3_moveit_control/wrist_path_constraints.hpp"
#include "robot_arm3_moveit_control/wrist_turn_preserver.hpp"

namespace robot_arm3_moveit_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kFrameBase = 0;
constexpr int kFrameTool = 1;
constexpr int kFrameUser = 2;
// Cartesian PATH blocks carry the user frame that was active when they were
// recorded.  Index 1 is local to each PathBlock, so G and H can safely use
// the same index in different blocks.
constexpr std::uint8_t kRecordedUserFrameIndex = 1;

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

int integer_mode(double value, const std::string& name)
{
  const int mode = static_cast<int>(std::round(value));
  if (!std::isfinite(value) || std::abs(value - mode) > 1e-9)
    throw std::runtime_error(name + " must be an integer");
  return mode;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z);
  if (orientation.norm() < 1e-12)
    throw std::runtime_error("Pose quaternion has zero length");
  orientation.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = orientation.toRotationMatrix();
  return transform;
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

double rounded_percent(double value, const std::string& name)
{
  if (!std::isfinite(value) || value < 1.0 || value > 100.0)
    throw std::runtime_error(name + " must be within [1, 100]");
  return std::round(value);
}

std::string planning_failure_message(const std::string& motion,
                                     const moveit::core::MoveItErrorCode& error)
{
  std::string reason;
  switch (error.val)
  {
    case moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION:
      reason = "the start state is in collision";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION:
      reason = "the goal state is in collision";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
      reason = "the generated trajectory contains a collision or another invalid state";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
      reason = "no inverse-kinematics solution exists for the target";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
      reason = "planning timed out before finding a valid path";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE:
      reason = "the current robot state is stale";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      reason = "a required coordinate-frame transform failed";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      reason = "collision checking is unavailable";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      reason = "the environment changed and invalidated the trajectory";
      break;
    case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
      reason = "no valid path was found";
      break;
    default:
      reason = moveit::core::error_code_to_string(error);
      break;
  }

  return motion + " planning failed: " + reason + " (MoveIt error " + std::to_string(error.val) + ")";
}

}  // namespace

class MoveItCommandServer : public rclcpp::Node
{
public:
  using ExecuteCommand = arm_tcp_bridge_interfaces::action::ExecuteCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCommand>;
  using ExecutePathAction = arm_tcp_bridge_interfaces::action::ExecutePath;
  using PathGoalHandle = rclcpp_action::ServerGoalHandle<ExecutePathAction>;
  using SignalPath = arm_tcp_bridge_interfaces::srv::SignalPath;
  using GetPathState = arm_tcp_bridge_interfaces::srv::GetPathState;
  using PathEvent = arm_tcp_bridge_interfaces::msg::PathEvent;
  using SequenceAction = moveit_msgs::action::MoveGroupSequence;
  using SequenceGoalHandle = rclcpp_action::ClientGoalHandle<SequenceAction>;

  MoveItCommandServer() : Node("moveit_program_command_server")
  {
    declare_parameter<std::string>("action_name", "/sim/arm/execute");
    declare_parameter<std::string>("path_action_name", "/sim/arm/execute_path");
    declare_parameter<std::string>("path_event_topic", "/sim/arm/path_events");
    declare_parameter<std::string>("prepared_path_topic", "/offline/prepared_path");
    declare_parameter<std::string>("motion_active_topic", "/offline/motion_active");
    declare_parameter<int>("max_path_nodes", 1000);
    declare_parameter<std::string>("flange_link", "Link_6");
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("end_effector_link", "Link_6");
    declare_parameter<std::string>("pilz_pipeline_id", "pilz_industrial_motion_planner");
    declare_parameter<std::string>("ptp_planner_id", "PTP");
    declare_parameter<std::string>("lin_planner_id", "LIN");
    declare_parameter<std::string>("circ_planner_id", "CIRC");
    declare_parameter<double>("pilz_max_trans_velocity", 1.0);
    declare_parameter<std::string>("ompl_pipeline_id", "ompl");
    declare_parameter<std::string>("ompl_planner_id", "RRTConnectkConfigDefault");
    declare_parameter<double>("ompl_planning_time", 10.0);
    declare_parameter<int>("ompl_planning_attempts", 10);
    declare_parameter<double>("ompl_wrist_corridor_margin_degrees", 0.5);
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
    ompl_pipeline_id_ = get_parameter("ompl_pipeline_id").as_string();
    ompl_planner_id_ = get_parameter("ompl_planner_id").as_string();
    ompl_planning_time_ = get_parameter("ompl_planning_time").as_double();
    ompl_planning_attempts_ = get_parameter("ompl_planning_attempts").as_int();
    ompl_wrist_corridor_margin_ =
        degrees_to_radians(get_parameter("ompl_wrist_corridor_margin_degrees").as_double());
    if (ompl_pipeline_id_.empty())
      throw std::runtime_error("ompl_pipeline_id cannot be empty");
    if (ompl_planner_id_.empty())
      throw std::runtime_error("ompl_planner_id cannot be empty");
    if (!std::isfinite(ompl_planning_time_) || ompl_planning_time_ <= 0.0)
      throw std::runtime_error("ompl_planning_time must be greater than zero");
    if (ompl_planning_attempts_ <= 0)
      throw std::runtime_error("ompl_planning_attempts must be greater than zero");
    if (!std::isfinite(ompl_wrist_corridor_margin_) || ompl_wrist_corridor_margin_ <= 0.0)
      throw std::runtime_error("ompl_wrist_corridor_margin_degrees must be greater than zero");
    const int max_fly_points = get_parameter("max_fly_points").as_int();
    if (max_fly_points <= 0)
      throw std::runtime_error("max_fly_points must be greater than zero");
    fly_queue_ = std::make_unique<FlyQueue>(static_cast<std::size_t>(max_fly_points));
    const int max_path_nodes = get_parameter("max_path_nodes").as_int();
    if (max_path_nodes <= 0)
      throw std::runtime_error("max_path_nodes must be greater than zero");
    max_path_nodes_ = static_cast<std::size_t>(max_path_nodes);
    flange_link_ = get_parameter("flange_link").as_string();
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

    path_action_server_ = rclcpp_action::create_server<ExecutePathAction>(
        this, get_parameter("path_action_name").as_string(),
        std::bind(&MoveItCommandServer::handle_path_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveItCommandServer::handle_path_cancel, this, std::placeholders::_1),
        std::bind(&MoveItCommandServer::handle_path_accepted, this, std::placeholders::_1));
    path_event_publisher_ = create_publisher<PathEvent>(
        get_parameter("path_event_topic").as_string(), rclcpp::QoS(20));
    prepared_path_publisher_ = create_publisher<arm_tcp_bridge_interfaces::msg::PathBlock>(
        get_parameter("prepared_path_topic").as_string(), rclcpp::QoS(10).transient_local());
    motion_active_publisher_ = create_publisher<std_msgs::msg::Bool>(
        get_parameter("motion_active_topic").as_string(), rclcpp::QoS(1).transient_local());
    publish_motion_active(false);
    signal_path_service_ = create_service<SignalPath>(
        "sim/arm/signal_path",
        std::bind(&MoveItCommandServer::signal_path, this, std::placeholders::_1, std::placeholders::_2));
    get_path_state_service_ = create_service<GetPathState>(
        "sim/arm/get_path_state",
        std::bind(&MoveItCommandServer::get_path_state, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Programmatic MoveIt server ready on %s", action_name.c_str());
    RCLCPP_INFO(get_logger(), "Programmatic PATH server ready on %s",
                get_parameter("path_action_name").as_string().c_str());
  }

private:
  enum class GoalTerminalState
  {
    SUCCEEDED,
    ABORTED,
    CANCELED,
  };

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
    try
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      if (active_sequence_goal_ && sequence_client_)
        sequence_client_->async_cancel_goal(active_sequence_goal_);
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Unable to forward sequence cancellation: %s", error.what());
    }
    try
    {
      if (move_group_)
        move_group_->stop();
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Unable to stop MoveIt execution: %s", error.what());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (busy_.exchange(true))
    {
      auto result = std::make_shared<ExecuteCommand::Result>();
      result->success = false;
      result->message = "Server became busy before execution";
      finish_goal(goal_handle, result, GoalTerminalState::ABORTED);
      return;
    }
    publish_motion_active(true);
    std::thread(&MoveItCommandServer::execute, this, goal_handle).detach();
  }

  rclcpp_action::GoalResponse handle_path_goal(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecutePathAction::Goal> goal)
  {
    if (busy_.load())
    {
      RCLCPP_WARN(get_logger(), "Rejecting PATH because the server is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    try
    {
      validate_path_goal(*goal, max_path_nodes_);
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Rejecting PATH: %s", error.what());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_path_cancel(const std::shared_ptr<PathGoalHandle>)
  {
    {
      std::lock_guard<std::mutex> lock(path_state_mutex_);
      path_cancel_requested_ = true;
      path_continue_requested_ = true;
      path_waiting_ = false;
      path_state_ = "CANCELING";
    }
    path_wait_condition_.notify_all();
    try
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      if (active_sequence_goal_ && sequence_client_)
        sequence_client_->async_cancel_goal(active_sequence_goal_);
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Unable to cancel active PATH sequence: %s", error.what());
    }
    try
    {
      if (move_group_)
        move_group_->stop();
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Unable to stop PATH execution: %s", error.what());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_path_accepted(const std::shared_ptr<PathGoalHandle> goal_handle)
  {
    if (busy_.exchange(true))
    {
      auto result = std::make_shared<ExecutePathAction::Result>();
      result->success = false;
      result->message = "Server became busy before PATH execution";
      result->executed_nodes = 0;
      goal_handle->abort(result);
      return;
    }
    publish_motion_active(true);
    std::thread(&MoveItCommandServer::execute_path_goal, this, goal_handle).detach();
  }

  void publish_path_feedback(const std::shared_ptr<PathGoalHandle>& goal_handle,
                             const std::string& state, std::uint32_t uploaded_nodes,
                             std::uint32_t current_node, bool waiting)
  {
    auto feedback = std::make_shared<ExecutePathAction::Feedback>();
    feedback->state = state;
    feedback->uploaded_nodes = uploaded_nodes;
    feedback->current_node = current_node;
    feedback->waiting = waiting;
    goal_handle->publish_feedback(feedback);
  }

  void publish_path_event(std::uint32_t path_id, std::uint32_t node_index,
                          std::uint8_t event_type, std::int32_t condition_handler,
                          const std::string& description)
  {
    PathEvent event;
    event.path_id = path_id;
    event.node_index = node_index;
    event.event_type = event_type;
    event.condition_handler = condition_handler;
    event.description = description;
    path_event_publisher_->publish(event);
  }

  void publish_path_conditions(const ExecutePathAction::Goal& goal,
                               const std::vector<std::uint32_t>& indexes, bool before)
  {
    std::map<std::uint8_t, std::int32_t> handlers;
    for (const auto& condition : goal.conditions)
      handlers[condition.slot] = condition.handler_id;
    const bool forward = indexes.size() < 2 || indexes.front() < indexes.back();
    for (const auto index : indexes)
    {
      const auto& node = goal.nodes.at(index - 1);
      publish_path_event(goal.path_id, index,
                         before ? PathEvent::NODE_START : PathEvent::NODE_END, 0,
                         before ? "node_start" : "node_end");
      const std::uint32_t mask = static_cast<std::uint32_t>(
          forward ? node.condition_mask : node.condition_mask_back);
      for (std::uint8_t bit = 0; bit < 32; ++bit)
      {
        if ((mask & (std::uint32_t{ 1 } << bit)) == 0)
          continue;
        const auto handler = handlers.find(static_cast<std::uint8_t>(bit + 1));
        if (handler == handlers.end())
          continue;
        const bool fire = (handler->second == 10 && before) ||
                          (handler->second == 11 && !before) ||
                          (handler->second == 12 && before &&
                           node.motion_type == arm_tcp_bridge_interfaces::msg::PathNode::CIRCULAR) ||
                          (handler->second != 10 && handler->second != 11 && handler->second != 12 && !before);
        if (fire)
          publish_path_event(goal.path_id, index,
                             handler->second == 12 ? PathEvent::VIA : PathEvent::CONDITION,
                             handler->second, "condition_handler");
      }
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan_path_sequence(
      const moveit_msgs::msg::MotionSequenceRequest& request,
      const std::shared_ptr<PathGoalHandle>& goal_handle)
  {
    if (!sequence_client_->wait_for_action_server(std::chrono::duration<double>(sequence_wait_timeout_)))
      throw std::runtime_error("Pilz sequence action server is not available");
    SequenceAction::Goal sequence_goal;
    sequence_goal.request = request;
    sequence_goal.planning_options.plan_only = true;
    rclcpp_action::Client<SequenceAction>::SendGoalOptions send_options;
    send_options.feedback_callback =
        [this, goal_handle](SequenceGoalHandle::SharedPtr,
                            const std::shared_ptr<const SequenceAction::Feedback> feedback) {
          publish_path_feedback(goal_handle, "pilz_sequence_" + feedback->state, 0,
                                active_path_node_, false);
        };
    auto goal_future = sequence_client_->async_send_goal(sequence_goal, send_options);
    if (goal_future.wait_for(std::chrono::duration<double>(sequence_wait_timeout_)) != std::future_status::ready)
      throw std::runtime_error("Timed out while sending PATH sequence goal");
    const auto sequence_handle = goal_future.get();
    if (!sequence_handle)
      throw std::runtime_error("Pilz rejected PATH sequence planning");
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      active_sequence_goal_ = sequence_handle;
    }
    auto result_future = sequence_client_->async_get_result(sequence_handle);
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
      if (goal_handle->is_canceling() || path_cancel_requested_)
      {
        sequence_client_->async_cancel_goal(sequence_handle);
        throw std::runtime_error("PATH planning canceled");
      }
    }
    const auto wrapped = result_future.get();
    {
      std::lock_guard<std::mutex> lock(sequence_goal_mutex_);
      active_sequence_goal_.reset();
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result)
      throw std::runtime_error("Pilz PATH sequence planning action failed");
    const auto& response = wrapped.result->response;
    if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      throw std::runtime_error(planning_failure_message(
          "Pilz PATH sequence", moveit::core::MoveItErrorCode(response.error_code)));
    if (response.planned_trajectories.size() != 1)
      throw std::runtime_error("Expected one combined trajectory for a PATH batch");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.start_state_ = response.sequence_start;
    plan.trajectory_ = response.planned_trajectories.front();
    plan.planning_time_ = response.planning_time;
    return plan;
  }

  void execute_path_goal(const std::shared_ptr<PathGoalHandle> goal_handle)
  {
    auto result = std::make_shared<ExecutePathAction::Result>();
    std::uint32_t executed_nodes = 0;
    try
    {
      const auto goal = *goal_handle->get_goal();
      validate_path_goal(goal, max_path_nodes_);
      {
        std::lock_guard<std::mutex> lock(path_state_mutex_);
        active_path_id_ = goal.path_id;
        active_path_node_ = 0;
        path_waiting_ = false;
        path_continue_requested_ = false;
        path_cancel_requested_ = false;
        path_state_ = "PLANNING";
      }
      const auto batches = partition_path_at_waits(goal);
      for (std::size_t batch_index = 0; batch_index < batches.size(); ++batch_index)
      {
        const auto& batch = batches[batch_index];
        if (goal_handle->is_canceling() || path_cancel_requested_)
          throw std::runtime_error("PATH canceled");
        active_path_node_ = batch.front();
        publish_path_feedback(goal_handle, "planning", 0, active_path_node_, false);
        const auto current_state = move_group_->getCurrentState(current_state_timeout_);
        if (!current_state)
          throw std::runtime_error("Unable to read current state for PATH planning");
        current_state->update();

        PathSequenceOptions options;
        options.planning_group = planning_group_;
        options.base_frame = base_frame_;
        options.end_effector_link = end_effector_link_;
        options.pipeline_id = pilz_pipeline_id_;
        options.ptp_planner_id = ptp_planner_id_;
        options.lin_planner_id = lin_planner_id_;
        options.circ_planner_id = circ_planner_id_;
        options.base_from_user = frame_transforms_.base_from_user();
        options.planning_time = planning_time_;
        options.planning_attempts = planning_attempts_;
        options.pilz_max_trans_velocity = pilz_max_trans_velocity_;
        options.default_linear_speed = speed_settings_.linear_speed_mps;
        options.coupling_offset = coupling_offset_;
        options.joint_3_min = joint_3_min_;
        options.joint_3_max = joint_3_max_;
        options.joint_7_min = joint_7_min_;
        options.joint_7_max = joint_7_max_;
        options.normal_radius_safety_factor = fly_norm_radius_safety_factor_;
        if (!move_group_->getRobotModel()->hasLinkModel(flange_link_))
          throw std::runtime_error("Configured PATH flange_link is unknown");
        options.flange_to_default_tool =
            current_state->getGlobalLinkTransform(flange_link_).inverse() *
            current_state->getGlobalLinkTransform(end_effector_link_);

        const auto request = build_path_sequence_batch(
            goal, batch, options, *current_state, move_group_->getRobotModel());
        auto plan = plan_path_sequence(request, goal_handle);
        const bool cartesian = goal.path_type == ExecutePathAction::Goal::CARTESIAN;
        (void)postprocess_trajectory(plan.trajectory_, cartesian, cartesian);
        publish_path_conditions(goal, batch, true);
        publish_path_feedback(goal_handle, "executing", 0, batch.front(), false);
        {
          std::lock_guard<std::mutex> lock(path_state_mutex_);
          path_state_ = "EXECUTING";
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
          throw std::runtime_error("MoveIt failed to execute PATH batch");
        publish_native_path(goal, batch, plan.trajectory_);
        executed_nodes += static_cast<std::uint32_t>(batch.size());
        active_path_node_ = batch.back();
        publish_path_conditions(goal, batch, false);

        const bool wait_here = goal.nodes.at(batch.back() - 1).wait &&
                               batch_index + 1 < batches.size();
        if (wait_here)
        {
          {
            std::lock_guard<std::mutex> lock(path_state_mutex_);
            path_waiting_ = true;
            path_continue_requested_ = false;
            path_state_ = "WAITING";
          }
          publish_path_event(goal.path_id, active_path_node_, PathEvent::WAITING, 0, "segment_wait");
          publish_path_feedback(goal_handle, "waiting", 0, active_path_node_, true);
          std::unique_lock<std::mutex> lock(path_state_mutex_);
          path_wait_condition_.wait(lock, [this, goal_handle] {
            return path_continue_requested_ || path_cancel_requested_ || goal_handle->is_canceling();
          });
          if (path_cancel_requested_ || goal_handle->is_canceling())
            throw std::runtime_error("PATH canceled while waiting");
          path_waiting_ = false;
          path_continue_requested_ = false;
          path_state_ = "EXECUTING";
          lock.unlock();
          publish_path_event(goal.path_id, active_path_node_, PathEvent::RESUMED, 0, "segment_resumed");
        }
      }
      {
        std::lock_guard<std::mutex> lock(path_state_mutex_);
        path_state_ = "DONE";
        path_waiting_ = false;
      }
      result->success = true;
      result->message = "PATH execution finished";
      result->executed_nodes = executed_nodes;
      goal_handle->succeed(result);
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(get_logger(), "PATH failed: %s", error.what());
      {
        std::lock_guard<std::mutex> lock(path_state_mutex_);
        path_state_ = goal_handle->is_canceling() || path_cancel_requested_ ? "CANCELED" : "ERROR";
        path_waiting_ = false;
      }
      result->success = false;
      result->message = error.what();
      result->executed_nodes = executed_nodes;
      if (goal_handle->is_canceling() || path_cancel_requested_)
        goal_handle->canceled(result);
      else
        goal_handle->abort(result);
    }
    busy_.store(false);
    publish_motion_active(false);
  }

  void signal_path(const std::shared_ptr<SignalPath::Request> request,
                   std::shared_ptr<SignalPath::Response> response)
  {
    std::lock_guard<std::mutex> lock(path_state_mutex_);
    if (request->path_id != active_path_id_ || !path_waiting_ ||
        (request->expected_node != 0 && request->expected_node != active_path_node_))
    {
      response->accepted = false;
      response->message = "PATH is not waiting at the requested node";
      return;
    }
    path_continue_requested_ = true;
    path_waiting_ = false;
    path_state_ = "EXECUTING";
    response->accepted = true;
    response->message = "PATH continue accepted";
    path_wait_condition_.notify_all();
  }

  void get_path_state(const std::shared_ptr<GetPathState::Request> request,
                      std::shared_ptr<GetPathState::Response> response)
  {
    std::lock_guard<std::mutex> lock(path_state_mutex_);
    response->known = request->path_id == active_path_id_ && active_path_id_ != 0;
    response->state = response->known ? path_state_ : "UNKNOWN";
    response->current_node = response->known ? active_path_node_.load() : 0;
    response->waiting = response->known && path_waiting_;
    response->message = response->known ? "PATH state available" : "PATH id is unknown";
  }

  void publish_feedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& state)
  {
    auto feedback = std::make_shared<ExecuteCommand::Feedback>();
    feedback->state = state;
    try
    {
      goal_handle->publish_feedback(feedback);
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Unable to publish command feedback '%s': %s", state.c_str(), error.what());
    }
  }

  void publish_motion_active(bool active)
  {
    if (!motion_active_publisher_)
      return;
    std_msgs::msg::Bool message;
    message.data = active;
    motion_active_publisher_->publish(message);
  }

  void finish_goal(const std::shared_ptr<GoalHandle>& goal_handle,
                   const std::shared_ptr<ExecuteCommand::Result>& result, GoalTerminalState state) noexcept
  {
    try
    {
      switch (state)
      {
        case GoalTerminalState::SUCCEEDED:
          goal_handle->succeed(result);
          break;
        case GoalTerminalState::ABORTED:
          goal_handle->abort(result);
          break;
        case GoalTerminalState::CANCELED:
          goal_handle->canceled(result);
          break;
      }
    }
    catch (const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Command completed but its Action result could not be delivered: %s", error.what());
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<ExecuteCommand::Result>();
    bool command_succeeded = false;
    try
    {
      const ParsedCommand command = parse_command(goal_handle->get_goal()->command);
      result->message = dispatch(command, goal_handle);
      command_succeeded = true;
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(get_logger(), "Command failed: %s", error.what());
      result->message = error.what();
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Command canceled";
      finish_goal(goal_handle, result, GoalTerminalState::CANCELED);
    }
    else if (command_succeeded)
    {
      result->success = true;
      finish_goal(goal_handle, result, GoalTerminalState::SUCCEEDED);
    }
    else
    {
      result->success = false;
      finish_goal(goal_handle, result, GoalTerminalState::ABORTED);
    }
    busy_.store(false);
    publish_motion_active(false);
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
      case CommandType::GET_FLY_QUEUE:
        return get_fly_queue();
      case CommandType::GET_MOTION_SETTINGS:
        return get_motion_settings();
      case CommandType::SET_BASE:
        return set_base(command.values);
      case CommandType::SET_USER_FRAME:
        return set_user_frame(command.values);
      case CommandType::SET_TOOL:
        throw std::runtime_error(
            "Dynamic setTool is not supported; configure Link_6 -> tcp_link in robot_arm3_moveit.urdf.xacro "
            "before launching MoveIt");
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
      case CommandType::MOVE_JOINT_AUTO:
        return move_joint_auto(command.values, goal_handle);
      case CommandType::MOVE_POSE_AUTO:
        return move_pose_auto(command.values, goal_handle);
      case CommandType::MOVE_RELATIVE:
        return move_relative(command.values, goal_handle);
      case CommandType::MOVE_ABOUT:
        return move_about(command.values, goal_handle);
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

  std::string set_base(const std::vector<double>& values)
  {
    if (!fly_queue_->empty())
      throw std::runtime_error("Clear the FLY queue before changing the base frame");
    frame_transforms_.set_base(six_values(values, "setBase"));
    return "Base frame updated; Cartesian commands remain expressed in the active user frame";
  }

  std::string set_user_frame(const std::vector<double>& values)
  {
    if (!fly_queue_->empty())
      throw std::runtime_error("Clear the FLY queue before changing the user frame");
    frame_transforms_.set_user_frame(six_values(values, "setUframe"));
    return "User frame updated; Cartesian commands and getPose now use this frame";
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
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
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

  void select_ompl_planner()
  {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setPlanningPipelineId(ompl_pipeline_id_);
    move_group_->setPlannerId(ompl_planner_id_);
    move_group_->setPlanningTime(ompl_planning_time_);
    move_group_->setNumPlanningAttempts(ompl_planning_attempts_);
    const double velocity_scaling =
        *std::max_element(speed_settings_.joint_overrides_percent.begin(),
                          speed_settings_.joint_overrides_percent.end()) / 100.0;
    const double acceleration_scaling =
        std::min(speed_settings_.acceleration_percent, speed_settings_.deceleration_percent) / 100.0;
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    RCLCPP_INFO(get_logger(), "OMPL %s request scaling: velocity=%.3f, acceleration=%.3f",
                ompl_planner_id_.c_str(), velocity_scaling, acceleration_scaling);
  }

  std::map<std::string, double> configured_joint_target() const
  {
    std::vector<double> target_values;
    move_group_->getJointValueTarget(target_values);
    const auto* joint_model_group =
        move_group_->getRobotModel()->getJointModelGroup(planning_group_);
    if (!joint_model_group)
      throw std::runtime_error("MoveIt planning group is unavailable: " + planning_group_);

    const auto& variable_names = joint_model_group->getVariableNames();
    if (variable_names.size() != target_values.size())
      throw std::runtime_error("MoveIt returned an incomplete joint target");

    std::map<std::string, double> target;
    for (std::size_t index = 0; index < variable_names.size(); ++index)
      target.emplace(variable_names[index], target_values[index]);
    return target;
  }

  moveit_msgs::msg::Constraints constrain_ompl_wrist_path(
      const std::map<std::string, double>& target)
  {
    const moveit::core::RobotStatePtr current_state =
        move_group_->getCurrentState(current_state_timeout_);
    if (!current_state)
      throw std::runtime_error("Unable to read the current wrist state for OMPL planning");

    const double joint_4_start = current_state->getVariablePosition("joint_4");
    const double joint_6_start = current_state->getVariablePosition("joint_6");
    const double joint_4_target = target.at("joint_4");
    const double joint_6_target = target.at("joint_6");
    const auto constraints = make_wrist_corridor_constraints(
        joint_4_start, joint_4_target,
        joint_6_start, joint_6_target,
        ompl_wrist_corridor_margin_);
    move_group_->setPathConstraints(constraints);

    RCLCPP_INFO(
        get_logger(),
        "OMPL wrist corridor: joint_4 %.3f -> %.3f deg, joint_6 %.3f -> %.3f deg, margin %.3f deg",
        radians_to_degrees(joint_4_start), radians_to_degrees(joint_4_target),
        radians_to_degrees(joint_6_start), radians_to_degrees(joint_6_target),
        radians_to_degrees(ompl_wrist_corridor_margin_));
    return constraints;
  }

  geometry_msgs::msg::Pose cartesian_pose(const std::vector<double>& values, std::size_t offset) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = values.at(offset) / 1000.0;
    pose.position.y = values.at(offset + 1) / 1000.0;
    pose.position.z = values.at(offset + 2) / 1000.0;
    pose.orientation = aer_to_quaternion(values.at(offset + 3), values.at(offset + 4), values.at(offset + 5));
    return frame_transforms_.command_to_base(pose);
  }

  geometry_msgs::msg::Pose current_tcp_pose() const
  {
    return move_group_->getCurrentPose(end_effector_link_).pose;
  }

  Eigen::Vector3d vector_in_base(const Eigen::Vector3d& vector, int frame_mode,
                                 const geometry_msgs::msg::Pose& current_pose) const
  {
    if (frame_mode == kFrameBase)
      return vector;
    if (frame_mode == kFrameTool)
      return pose_to_eigen(current_pose).linear() * vector;
    if (frame_mode == kFrameUser)
      return frame_transforms_.base_from_user().linear() * vector;

    throw std::runtime_error("Frame mode must be 0 (BASE), 1 (TOOL) or 2 (UFRAME)");
  }

  std::string execute_linear_pose_target(const geometry_msgs::msg::Pose& target,
                                         const std::shared_ptr<GoalHandle>& goal_handle,
                                         const std::string& motion_name,
                                         const std::string& feedback_prefix,
                                         const std::string& path_name,
                                         const std::vector<double>& path_target)
  {
    select_pilz_planner(lin_planner_id_, true);
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target, end_effector_link_);

    publish_feedback(goal_handle, "planning_" + feedback_prefix);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error(planning_failure_message(motion_name, planning_result));

    publish_feedback(goal_handle, "validating_and_retiming_" + feedback_prefix);
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, true, true);

    if (goal_handle->is_canceling())
      return "Command canceled before execution";

    publish_feedback(goal_handle, "executing_" + feedback_prefix);
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the " + motion_name + " trajectory");
    publish_command_path(
        plan.trajectory_, path_name,
        ExecutePathAction::Goal::CARTESIAN,
        { command_path_node(
            arm_tcp_bridge_interfaces::msg::PathNode::LINEAR,
            path_target, kRecordedUserFrameIndex) });

    return motion_name + " finished; " + scaling_description(scaling);
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

  void publish_ompl_path(const moveit_msgs::msg::RobotTrajectory& trajectory,
                         const std::string& name)
  {
    TrajectoryPathOptions options;
    options.coupling_offset = coupling_offset_;
    options.joint_3_min = joint_3_min_;
    options.joint_3_max = joint_3_max_;
    options.joint_7_min = joint_7_min_;
    options.joint_7_max = joint_7_max_;
    options.max_nodes = max_path_nodes_;
    options.segment_override = 5.0;
    options.omit_initial_node = true;
    auto block = trajectory_to_joint_path(trajectory, options);
    if (block.nodes.empty())
    {
      RCLCPP_INFO(
          get_logger(),
          "OMPL trajectory for %s contains no motion after its start state; "
          "no PATH draft was published",
          name.c_str());
      return;
    }
    block.name = name;
    prepared_path_publisher_->publish(block);
    RCLCPP_INFO(get_logger(), "Published prepared C4G JOINT PATH for %s with %zu nodes",
                name.c_str(), block.nodes.size());
  }

  arm_tcp_bridge_interfaces::msg::PathNode command_path_node(
      std::uint8_t motion_type, const std::vector<double>& target,
      std::uint8_t reference_index = 0) const
  {
    arm_tcp_bridge_interfaces::msg::PathNode node;
    node.motion_type = motion_type;
    node.target = six_values(target, "PATH node target");
    node.linear_speed = speed_settings_.linear_speed_mps;
    node.rotational_speed = 10.0;
    node.segment_override =
        *std::max_element(speed_settings_.joint_overrides_percent.begin(),
                          speed_settings_.joint_overrides_percent.end());
    node.termination_type = 1;
    node.tolerance = 1.0;
    node.segment_data = true;
    node.fly = false;
    node.fly_type = 0;
    node.fly_percent = 75.0;
    node.fly_distance_mm = 0.0;
    node.fly_trajectory = 0;
    node.stress_percent = 0.0;
    node.reference_index = reference_index;
    node.tool_index = 0;
    node.condition_mask = 0;
    node.condition_mask_back = 0;
    node.wait = false;
    return node;
  }

  arm_tcp_bridge_interfaces::msg::PathFrame recorded_user_frame() const
  {
    arm_tcp_bridge_interfaces::msg::PathFrame frame;
    frame.index = kRecordedUserFrameIndex;
    frame.pose = frame_transforms_.user_frame_values();
    return frame;
  }

  arm_tcp_bridge_interfaces::msg::PathBlock command_path_block(
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& name, std::uint8_t path_type,
      std::vector<arm_tcp_bridge_interfaces::msg::PathNode> nodes) const
  {
    TrajectoryPathOptions options;
    options.coupling_offset = coupling_offset_;
    options.joint_3_min = joint_3_min_;
    options.joint_3_max = joint_3_max_;
    options.joint_7_min = joint_7_min_;
    options.joint_7_max = joint_7_max_;
    options.max_nodes = max_path_nodes_;
    const auto endpoints = trajectory_to_joint_path(trajectory, options);

    arm_tcp_bridge_interfaces::msg::PathBlock block;
    block.name = name;
    block.path_type = path_type;
    if (path_type == ExecutePathAction::Goal::CARTESIAN)
      block.frames.push_back(recorded_user_frame());
    block.nodes = std::move(nodes);
    block.start_index = 1;
    block.end_index = static_cast<std::uint32_t>(block.nodes.size());
    block.expected_start_deg = endpoints.expected_start_deg;
    block.expected_end_deg = endpoints.expected_end_deg;
    block.wait_after = false;
    return block;
  }

  void publish_command_path(
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& name, std::uint8_t path_type,
      std::vector<arm_tcp_bridge_interfaces::msg::PathNode> nodes)
  {
    auto block = command_path_block(
        trajectory, name, path_type, std::move(nodes));
    prepared_path_publisher_->publish(block);
    RCLCPP_INFO(get_logger(), "Published prepared native %s with %zu nodes",
                name.c_str(), block.nodes.size());
  }

  std::vector<double> command_values_from_base_pose(
      const geometry_msgs::msg::Pose& base_pose) const
  {
    const auto user_pose = frame_transforms_.base_to_user(base_pose);
    const auto aer = quaternion_to_aer(user_pose.orientation);
    return {
      user_pose.position.x * 1000.0,
      user_pose.position.y * 1000.0,
      user_pose.position.z * 1000.0,
      aer.x(),
      aer.y(),
      aer.z(),
    };
  }

  void publish_fly_path(
      const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    const bool joint = fly_queue_->type() == FlyQueueType::JOINT;
    const std::uint8_t reference_index =
        joint ? 0 : kRecordedUserFrameIndex;
    std::vector<arm_tcp_bridge_interfaces::msg::PathNode> nodes;
    for (std::size_t index = 0; index < fly_queue_->segments().size();
         ++index)
    {
      const auto& segment = fly_queue_->segments()[index];
      const bool fly = index + 1 < fly_queue_->segments().size();
      if (segment.type == FlySegmentType::CIRCULAR)
      {
        std::vector<double> via(segment.values.begin(),
                                segment.values.begin() + 6);
        std::vector<double> destination(segment.values.begin() + 6,
                                        segment.values.end());
        auto via_node = command_path_node(
            arm_tcp_bridge_interfaces::msg::PathNode::SEG_VIA, via,
            reference_index);
        nodes.push_back(via_node);
        auto node = command_path_node(
            arm_tcp_bridge_interfaces::msg::PathNode::CIRCULAR,
            destination, reference_index);
        node.fly = fly;
        nodes.push_back(node);
      }
      else
      {
        const auto motion_type =
            segment.type == FlySegmentType::JOINT
                ? arm_tcp_bridge_interfaces::msg::PathNode::JOINT
                : arm_tcp_bridge_interfaces::msg::PathNode::LINEAR;
        auto node = command_path_node(
            motion_type, segment.values, reference_index);
        node.fly = fly;
        nodes.push_back(node);
      }
      if (fly)
      {
        auto& node = nodes.back();
        if (fly_settings_.mode == FlyMode::CARTESIAN)
        {
          node.fly_type = 1;
          node.fly_distance_mm = fly_settings_.distance_mm;
          node.fly_trajectory =
              static_cast<std::uint8_t>(fly_settings_.trajectory_mode);
          node.stress_percent = fly_settings_.stress_percent;
          node.fly_percent = 100.0;
        }
        else
        {
          node.fly_type = 0;
          node.fly_percent = fly_settings_.normal_percent;
        }
      }
    }
    publish_command_path(
        trajectory, joint ? "executeFlyQueue_joint"
                          : "executeFlyQueue_cartesian",
        joint ? ExecutePathAction::Goal::JOINT
              : ExecutePathAction::Goal::CARTESIAN,
        std::move(nodes));
  }

  void publish_native_path(const ExecutePathAction::Goal& goal,
                           const std::vector<std::uint32_t>& indexes,
                           const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    TrajectoryPathOptions options;
    options.coupling_offset = coupling_offset_;
    options.joint_3_min = joint_3_min_;
    options.joint_3_max = joint_3_max_;
    options.joint_7_min = joint_7_min_;
    options.joint_7_max = joint_7_max_;
    options.max_nodes = max_path_nodes_;
    const auto endpoints = trajectory_to_joint_path(trajectory, options);

    arm_tcp_bridge_interfaces::msg::PathBlock block;
    block.name = "native_path_" + std::to_string(goal.path_id);
    block.path_id = goal.path_id;
    block.path_type = goal.path_type;
    block.frames = goal.frames;
    block.conditions = goal.conditions;
    block.expected_start_deg = endpoints.expected_start_deg;
    block.expected_end_deg = endpoints.expected_end_deg;
    for (const auto index : indexes)
      block.nodes.push_back(goal.nodes.at(index - 1));
    block.start_index = 1;
    block.end_index = static_cast<std::uint32_t>(block.nodes.size());
    block.wait_after = false;
    prepared_path_publisher_->publish(block);
    RCLCPP_INFO(get_logger(), "Published prepared native PATH %u with %zu nodes",
                goal.path_id, block.nodes.size());
  }

  std::map<std::string, double> joint_target_from_c4g_degrees(
      const std::vector<double>& degrees, const std::string& command_name) const
  {
    const double joint_1 = degrees_to_radians(degrees.at(0));
    const double joint_2 = degrees_to_radians(degrees.at(1));
    const double joint_3 = degrees_to_radians(degrees.at(2));
    const double joint_4 = degrees_to_radians(degrees.at(3));
    const double joint_5 = degrees_to_radians(degrees.at(4));
    const double joint_6 = degrees_to_radians(degrees.at(5));
    const double joint_7 = joint_2 + joint_3 + coupling_offset_;

    if (joint_3 < joint_3_min_ || joint_3 > joint_3_max_)
      throw std::runtime_error(command_name + " target produces joint_3 outside its configured limits");
    if (joint_7 < joint_7_min_ || joint_7 > joint_7_max_)
      throw std::runtime_error(command_name + " target produces joint_7 outside [-66, 60] degrees");

    return { { "joint_1", joint_1 }, { "joint_2", joint_2 }, { "joint_7", joint_7 },
             { "joint_4", joint_4 }, { "joint_5", joint_5 }, { "joint_6", joint_6 } };
  }

  std::string move_joint(const std::vector<double>& degrees, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_joint_target");
    const auto target = joint_target_from_c4g_degrees(degrees, "moveJoint");
    select_pilz_planner(ptp_planner_id_, false);
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(target))
      throw std::runtime_error("MoveIt rejected the requested joint target");

    publish_feedback(goal_handle, "planning_pilz_ptp");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error(planning_failure_message("Pilz PTP", planning_result));
    publish_feedback(goal_handle, "validating_and_retiming_pilz_ptp");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, false, false);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";

    publish_feedback(goal_handle, "executing_joint_motion");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the joint trajectory");
    publish_command_path(
        plan.trajectory_, "moveJoint", ExecutePathAction::Goal::JOINT,
        { command_path_node(
            arm_tcp_bridge_interfaces::msg::PathNode::JOINT, degrees) });
    return "Joint motion finished; " + scaling_description(scaling);
  }

  std::string move_joint_auto(const std::vector<double>& degrees,
                              const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_ompl_joint_target");
    const auto target = joint_target_from_c4g_degrees(degrees, "moveJointAuto");
    select_ompl_planner();
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(target))
      throw std::runtime_error("MoveIt rejected the automatic joint target");
    const auto wrist_constraints = constrain_ompl_wrist_path(target);

    publish_feedback(goal_handle, "planning_ompl_joint_avoidance");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPathConstraints();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error(planning_failure_message("OMPL joint obstacle-avoidance", planning_result));

    publish_feedback(goal_handle, "validating_and_retiming_ompl_joint_avoidance");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, false, false);
    validate_wrist_corridor_trajectory(plan.trajectory_, wrist_constraints);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";

    publish_feedback(goal_handle, "executing_ompl_joint_avoidance");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the OMPL joint obstacle-avoidance trajectory");
    publish_ompl_path(plan.trajectory_, "moveJointAuto");
    return "Automatic joint obstacle-avoidance motion finished; " + scaling_description(scaling);
  }

  std::string move_linear(const std::vector<double>& values, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_linear_target");
    const geometry_msgs::msg::Pose target = cartesian_pose(values, 0);
    return execute_linear_pose_target(
        target, goal_handle, "Linear motion", "pilz_lin", "moveLin",
        values);
  }

  std::string move_pose_auto(const std::vector<double>& values,
                             const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_ompl_pose_target");
    const geometry_msgs::msg::Pose target = cartesian_pose(values, 0);
    select_ompl_planner();
    move_group_->setStartStateToCurrentState();
    // Resolve one IK state from the current joint target before planning.
    // A pose goal lets OMPL sample any equivalent IK solution, which is
    // undesirable for the multi-turn wrist joints because it can select a
    // needlessly distant absolute turn.
    if (!move_group_->setJointValueTarget(target, end_effector_link_))
      throw std::runtime_error("No inverse-kinematics solution exists for the automatic pose target");
    const auto wrist_constraints =
        constrain_ompl_wrist_path(configured_joint_target());

    publish_feedback(goal_handle, "planning_ompl_pose_avoidance");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    move_group_->clearPathConstraints();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error(planning_failure_message("OMPL pose obstacle-avoidance", planning_result));

    publish_feedback(goal_handle, "validating_and_retiming_ompl_pose_avoidance");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, true, true);
    validate_wrist_corridor_trajectory(plan.trajectory_, wrist_constraints);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";

    publish_feedback(goal_handle, "executing_ompl_pose_avoidance");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the OMPL pose obstacle-avoidance trajectory");
    publish_ompl_path(plan.trajectory_, "movePoseAuto");
    return "Automatic pose obstacle-avoidance motion finished; " + scaling_description(scaling);
  }

  std::string move_relative(const std::vector<double>& values, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_relative_target");
    const int frame_mode = integer_mode(values.at(3), "moveRelative frame");
    const Eigen::Vector3d relative_vector(values.at(0) / 1000.0, values.at(1) / 1000.0, values.at(2) / 1000.0);
    if (relative_vector.norm() < 1e-12)
      throw std::runtime_error("moveRelative vector must not be zero");

    geometry_msgs::msg::Pose target = current_tcp_pose();
    const Eigen::Vector3d delta_base = vector_in_base(relative_vector, frame_mode, target);
    target.position.x += delta_base.x();
    target.position.y += delta_base.y();
    target.position.z += delta_base.z();

    return execute_linear_pose_target(
        target, goal_handle, "Relative motion", "pilz_relative",
        "moveRelative",
        command_values_from_base_pose(target));
  }

  std::string move_about(const std::vector<double>& values, const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publish_feedback(goal_handle, "validating_about_target");
    const int frame_mode = integer_mode(values.at(4), "moveAbout frame");
    const Eigen::Vector3d raw_axis(values.at(0), values.at(1), values.at(2));
    if (raw_axis.norm() < 1e-12)
      throw std::runtime_error("moveAbout vector must not be zero");

    const double angle_radians = degrees_to_radians(values.at(3));
    if (!std::isfinite(angle_radians) || std::abs(angle_radians) < 1e-12)
      throw std::runtime_error("moveAbout angle must not be zero");

    const geometry_msgs::msg::Pose current_pose = current_tcp_pose();
    const Eigen::Vector3d axis_base = vector_in_base(raw_axis.normalized(), frame_mode, current_pose).normalized();
    Eigen::Isometry3d target_transform = pose_to_eigen(current_pose);
    target_transform.linear() = (Eigen::AngleAxisd(angle_radians, axis_base) * target_transform.linear()).eval();
    const geometry_msgs::msg::Pose target = eigen_to_pose(target_transform);

    return execute_linear_pose_target(
        target, goal_handle, "About motion", "pilz_about", "moveAbout",
        command_values_from_base_pose(target));
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
      throw std::runtime_error(planning_failure_message("Pilz CIRC", planning_result));

    publish_feedback(goal_handle, "validating_and_retiming_pilz_circ");
    const ScalingResult scaling = postprocess_trajectory(plan.trajectory_, true, true);
    if (goal_handle->is_canceling())
      return "Command canceled before execution";
    publish_feedback(goal_handle, "executing_pilz_circ");
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("MoveIt failed to execute the Pilz CIRC trajectory");
    std::vector<double> via(values.begin(), values.begin() + 6);
    std::vector<double> destination(values.begin() + 6, values.end());
    publish_command_path(
        plan.trajectory_, "moveCircular",
        ExecutePathAction::Goal::CARTESIAN,
        {
          command_path_node(
              arm_tcp_bridge_interfaces::msg::PathNode::SEG_VIA, via,
              kRecordedUserFrameIndex),
          command_path_node(
              arm_tcp_bridge_interfaces::msg::PathNode::CIRCULAR,
              destination, kRecordedUserFrameIndex),
        });
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
    options.base_from_user = frame_transforms_.base_from_user();
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
      throw std::runtime_error(planning_failure_message(
          "Pilz sequence", moveit::core::MoveItErrorCode(response.error_code)));
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
    publish_fly_path(plan.trajectory_);
    fly_queue_->clear();
    return "FLY sequence finished; segments=" + std::to_string(executed_count) + "; " +
           scaling_description(scaling);
  }

  std::string get_pose()
  {
    const geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose(end_effector_link_);
    const geometry_msgs::msg::Pose user_pose = frame_transforms_.base_to_user(pose.pose);
    const Eigen::Vector3d aer = quaternion_to_aer(user_pose.orientation);
    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "frame=user_frame, planning_frame=" << pose.header.frame_id
            << ", link=" << end_effector_link_ << ", X=" << user_pose.position.x * 1000.0
            << ", Y=" << user_pose.position.y * 1000.0 << ", Z=" << user_pose.position.z * 1000.0
            << ", A=" << aer.x() << ", E=" << aer.y() << ", R=" << aer.z();
    return message.str();
  }

  std::string get_fly_queue() const
  {
    std::string queue_type = "NONE";
    if (fly_queue_->type() == FlyQueueType::CARTESIAN)
      queue_type = "CARTESIAN";
    else if (fly_queue_->type() == FlyQueueType::JOINT)
      queue_type = "JOINT";

    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "type=" << queue_type << ", count=" << fly_queue_->size()
            << ", max=" << fly_queue_->max_points() << ", segments=[";
    for (std::size_t index = 0; index < fly_queue_->segments().size(); ++index)
    {
      if (index > 0)
        message << "; ";
      const FlySegment& segment = fly_queue_->segments()[index];
      std::string segment_type = "JOINT";
      if (segment.type == FlySegmentType::LINEAR)
        segment_type = "LIN";
      else if (segment.type == FlySegmentType::CIRCULAR)
        segment_type = "CIRC";
      message << index + 1 << ":" << segment_type << "(";
      for (std::size_t value_index = 0; value_index < segment.values.size(); ++value_index)
      {
        if (value_index > 0)
          message << ",";
        message << segment.values[value_index];
      }
      message << ")";
    }
    message << "]; units=JOINT[deg], LIN/CIRC[mm,deg]";
    return message.str();
  }

  std::string get_motion_settings() const
  {
    std::ostringstream message;
    message << std::fixed << std::setprecision(6) << "orientation=RS_WORLD, joint_overrides_percent=[";
    for (std::size_t axis = 0; axis < speed_settings_.joint_overrides_percent.size(); ++axis)
    {
      if (axis > 0)
        message << ",";
      message << speed_settings_.joint_overrides_percent[axis];
    }
    message << "], physical_joint_max_velocities_rad_s=[";
    for (std::size_t axis = 0; axis < speed_settings_.max_joint_velocities.size(); ++axis)
    {
      if (axis > 0)
        message << ",";
      message << speed_settings_.max_joint_velocities[axis];
    }
    message << "], physical_joint_max_accelerations_rad_s2=[";
    for (std::size_t axis = 0; axis < speed_settings_.max_joint_accelerations.size(); ++axis)
    {
      if (axis > 0)
        message << ",";
      message << speed_settings_.max_joint_accelerations[axis];
    }
    message << "], linear_speed_mps=" << speed_settings_.linear_speed_mps
            << ", acceleration_percent=" << speed_settings_.acceleration_percent
            << ", deceleration_percent=" << speed_settings_.deceleration_percent
            << ", planning_frame=" << base_frame_ << ", end_effector_link=" << end_effector_link_;

    const auto append_frame = [&message](const char* name, const std::array<double, 6>& values) {
      message << ", " << name << "_mm_deg=[";
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        if (index > 0)
          message << ",";
        message << values[index];
      }
      message << "]";
    };
    append_frame("base", frame_transforms_.base_values());
    append_frame("user_frame", frame_transforms_.user_frame_values());

    if (fly_settings_.mode == FlyMode::CARTESIAN)
    {
      message << ", fly_mode=CARTESIAN, fly_stress_percent=" << fly_settings_.stress_percent
              << ", fly_trajectory_mode=" << fly_settings_.trajectory_mode
              << ", fly_distance_mm=" << fly_settings_.distance_mm;
    }
    else
    {
      message << ", fly_mode=NORMAL, fly_normal_percent=" << fly_settings_.normal_percent
              << ", fly_norm_radius_safety_factor=" << fly_norm_radius_safety_factor_;
    }
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
  rclcpp_action::Server<ExecutePathAction>::SharedPtr path_action_server_;
  rclcpp_action::Client<SequenceAction>::SharedPtr sequence_client_;
  rclcpp::Publisher<PathEvent>::SharedPtr path_event_publisher_;
  rclcpp::Publisher<arm_tcp_bridge_interfaces::msg::PathBlock>::SharedPtr prepared_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_active_publisher_;
  rclcpp::Service<SignalPath>::SharedPtr signal_path_service_;
  rclcpp::Service<GetPathState>::SharedPtr get_path_state_service_;
  SequenceGoalHandle::SharedPtr active_sequence_goal_;
  std::mutex sequence_goal_mutex_;
  std::atomic_bool busy_{ false };
  std::mutex path_state_mutex_;
  std::condition_variable path_wait_condition_;
  std::uint32_t active_path_id_{ 0 };
  std::atomic_uint32_t active_path_node_{ 0 };
  bool path_waiting_{ false };
  bool path_continue_requested_{ false };
  std::atomic_bool path_cancel_requested_{ false };
  std::string path_state_{ "EMPTY" };
  std::size_t max_path_nodes_{ 1000 };
  std::string flange_link_{ "Link_6" };
  std::string planning_group_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string pilz_pipeline_id_;
  std::string ptp_planner_id_;
  std::string lin_planner_id_;
  std::string circ_planner_id_;
  std::string ompl_pipeline_id_;
  std::string ompl_planner_id_;
  double current_state_timeout_{ 2.0 };
  MotionSpeedSettings speed_settings_;
  FrameTransformManager frame_transforms_;
  std::unique_ptr<FlyQueue> fly_queue_;
  FlySettings fly_settings_;
  double pilz_max_trans_velocity_{ 1.0 };
  double fly_norm_radius_safety_factor_{ 0.45 };
  double sequence_wait_timeout_{ 10.0 };
  double planning_time_{ 5.0 };
  int planning_attempts_{ 1 };
  double ompl_planning_time_{ 10.0 };
  int ompl_planning_attempts_{ 10 };
  double ompl_wrist_corridor_margin_{ 0.008726646259971648 };
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
    while (rclcpp::ok())
    {
      try
      {
        executor.spin_once(std::chrono::milliseconds(100));
      }
      catch (const rclcpp::exceptions::RCLError& error)
      {
        if (rclcpp::ok())
          RCLCPP_WARN(node->get_logger(), "Recoverable ROS communication error: %s", error.what());
      }
    }
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
