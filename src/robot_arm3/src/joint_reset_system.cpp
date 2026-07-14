#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>

#include <arm_tcp_bridge_interfaces/srv/reset_joints.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace robot_arm3
{
class JointResetSystem final : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure,
                               public ignition::gazebo::ISystemPreUpdate
{
public:
  using ResetJoints = arm_tcp_bridge_interfaces::srv::ResetJoints;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  enum class ResetPhase
  {
    IDLE,
    REQUEST_PAUSE,
    WAIT_PAUSED,
    WAIT_UNPAUSED,
  };

  ~JointResetSystem() override
  {
    if (executor_)
      executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
  }

  void Configure(const ignition::gazebo::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 ignition::gazebo::EntityComponentManager&,
                 ignition::gazebo::EventManager& event_manager) override
  {
    model_ = ignition::gazebo::Model(entity);
    event_manager_ = &event_manager;
    std::string service_name = "/gazebo/reset_robot_joints";
    std::string mirror_service_name = "/gazebo/reset_robot_joints_for_mirror";
    std::string switch_service = "/controller_manager/switch_controller";
    std::string list_service = "/controller_manager/list_controllers";
    controller_names_ = {"arm_controller", "internal_passive_controller"};
    reset_apply_tolerance_ = 0.001;
    reset_settle_duration_s_ = 0.1;
    reset_verify_timeout_s_ = 1.0;
    if (sdf && sdf->HasElement("reset_service"))
      service_name = sdf->Get<std::string>("reset_service");
    if (sdf && sdf->HasElement("mirror_reset_service"))
      mirror_service_name = sdf->Get<std::string>("mirror_reset_service");
    if (sdf && sdf->HasElement("controller_switch_service"))
      switch_service = sdf->Get<std::string>("controller_switch_service");
    if (sdf && sdf->HasElement("controller_list_service"))
      list_service = sdf->Get<std::string>("controller_list_service");
    if (sdf && sdf->HasElement("trajectory_controller"))
      controller_names_[0] = sdf->Get<std::string>("trajectory_controller");
    if (sdf && sdf->HasElement("passive_controller"))
      controller_names_[1] = sdf->Get<std::string>("passive_controller");
    if (sdf && sdf->HasElement("reset_apply_tolerance"))
      reset_apply_tolerance_ = sdf->Get<double>("reset_apply_tolerance");
    if (sdf && sdf->HasElement("reset_settle_duration_s"))
      reset_settle_duration_s_ = sdf->Get<double>("reset_settle_duration_s");
    if (sdf && sdf->HasElement("reset_verify_timeout_s"))
      reset_verify_timeout_s_ = sdf->Get<double>("reset_verify_timeout_s");

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("robot_arm3_joint_reset_system");
    switch_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    switch_client_ = node_->create_client<SwitchController>(
        switch_service, rmw_qos_profile_services_default,
        switch_callback_group_);
    list_client_ = node_->create_client<ListControllers>(
        list_service, rmw_qos_profile_services_default,
        switch_callback_group_);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 2);
    service_ = node_->create_service<ResetJoints>(
        service_name, [this](const std::shared_ptr<ResetJoints::Request> request,
                             std::shared_ptr<ResetJoints::Response> response) {
          HandleResetRequest(request, response, true);
        });
    mirror_service_ = node_->create_service<ResetJoints>(
        mirror_service_name, [this](const std::shared_ptr<ResetJoints::Request> request,
                                    std::shared_ptr<ResetJoints::Response> response) {
          HandleResetRequest(request, response, false);
        });
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });
    RCLCPP_INFO(
        node_->get_logger(),
        "Joint reset services ready: %s uses a paused world; %s resets in PreUpdate without pausing physics",
        service_name.c_str(), mirror_service_name.c_str());
  }

  void HandleResetRequest(
      const std::shared_ptr<ResetJoints::Request> request,
      std::shared_ptr<ResetJoints::Response> response,
      const bool pause_world)
  {
          std::lock_guard<std::mutex> service_lock(service_mutex_);
          if (request->joint_names.empty() ||
              request->joint_names.size() != request->positions.size())
          {
            response->success = false;
            response->message = "joint_names and positions must have the same non-zero length";
            return;
          }
          if (!AllFinite(request->positions))
          {
            response->success = false;
            response->message = "joint reset positions must be finite";
            return;
          }

          std::vector<std::string> controllers_to_restore;
          std::string controller_error;
          if (!ActiveResetControllers(controllers_to_restore, controller_error))
          {
            response->success = false;
            response->message = "failed to query reset controllers: " + controller_error;
            return;
          }
          if (!SwitchControllers({}, controllers_to_restore, controller_error))
          {
            response->success = false;
            response->message = "failed to deactivate reset controllers: " + controller_error;
            return;
          }

          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (phase_ != ResetPhase::IDLE)
            {
              std::string restore_error;
              SwitchControllers(controllers_to_restore, {}, restore_error);
              response->success = false;
              response->message = "another joint reset is already pending";
              return;
            }
            names_ = request->joint_names;
            positions_ = request->positions;
            completed_ = false;
            completion_success_ = false;
            completion_message_.clear();
            reset_applied_ = false;
            reset_apply_error_.clear();
            unpaused_time_started_ = false;
            pause_world_ = pause_world;
            phase_ = ResetPhase::REQUEST_PAUSE;
          }

          std::unique_lock<std::mutex> lock(mutex_);
          if (!condition_.wait_for(lock, std::chrono::seconds(5),
                                   [this] { return completed_; }))
          {
            phase_ = ResetPhase::IDLE;
            completion_success_ = false;
            completion_message_ =
                "timed out waiting for Gazebo to apply atomic joint reset";
            if (pause_world_ && event_manager_)
              event_manager_->Emit<ignition::gazebo::events::Pause>(false);
          }
          const bool reset_success = completion_success_;
          const std::string reset_message = completion_message_;
          lock.unlock();

          std::string activation_error;
          const bool activated = SwitchControllers(
              controllers_to_restore, {}, activation_error);
          response->success = reset_success && activated;
          response->message = reset_message;
          if (!activated)
          {
            response->message +=
                "; failed to reactivate reset controllers: " + activation_error;
          }
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                 ignition::gazebo::EntityComponentManager& ecm) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ == ResetPhase::IDLE)
      return;

    if (phase_ == ResetPhase::REQUEST_PAUSE)
    {
      if (pause_world_)
      {
        event_manager_->Emit<ignition::gazebo::events::Pause>(true);
        phase_ = ResetPhase::WAIT_PAUSED;
        return;
      }

      std::string error;
      if (!ApplyReset(ecm, error))
      {
        reset_applied_ = false;
        reset_apply_error_ = error;
      }
      else
      {
        reset_applied_ = true;
      }
      ClearLinkVelocities(ecm);
      unpaused_start_sim_time_ = info.simTime;
      unpaused_time_started_ = true;
      phase_ = ResetPhase::WAIT_UNPAUSED;
      return;
    }

    if (phase_ == ResetPhase::WAIT_PAUSED)
    {
      if (!info.paused)
        return;

      std::string error;
      if (!ApplyReset(ecm, error))
      {
        reset_applied_ = false;
        reset_apply_error_ = error;
      }
      else
      {
        reset_applied_ = true;
      }
      ClearLinkVelocities(ecm);
      event_manager_->Emit<ignition::gazebo::events::Pause>(false);
      phase_ = ResetPhase::WAIT_UNPAUSED;
      return;
    }

    if (phase_ != ResetPhase::WAIT_UNPAUSED || (pause_world_ && info.paused))
      return;

    if (!reset_applied_)
    {
      CompleteReset(false, reset_apply_error_);
      return;
    }

    ClearLinkVelocities(ecm);
    if (!unpaused_time_started_)
    {
      unpaused_start_sim_time_ = info.simTime;
      unpaused_time_started_ = true;
    }
    const double unpaused_duration_s =
        std::chrono::duration<double>(
            info.simTime - unpaused_start_sim_time_).count();
    if (unpaused_duration_s < reset_settle_duration_s_)
      return;

    std::string verification_error;
    if (ResetMatches(ecm, verification_error))
    {
      CompleteReset(
          true,
          "joint positions, joint velocities, and link velocities reset");
      return;
    }

    if (unpaused_duration_s >= reset_verify_timeout_s_)
      CompleteReset(false, verification_error);
  }

private:
  static bool AllFinite(const std::vector<double>& values)
  {
    for (const double value : values)
    {
      if (!std::isfinite(value))
        return false;
    }
    return true;
  }

  bool ActiveResetControllers(std::vector<std::string>& active,
                              std::string& error)
  {
    using namespace std::chrono_literals;
    if (!list_client_->wait_for_service(1s))
    {
      error = "controller list service is unavailable";
      return false;
    }

    auto future = list_client_->async_send_request(
        std::make_shared<ListControllers::Request>());
    if (future.wait_for(3s) != std::future_status::ready)
    {
      error = "controller list request timed out";
      return false;
    }
    const auto result = future.get();
    if (!result)
    {
      error = "controller list service returned no response";
      return false;
    }

    for (const auto& controller : result->controller)
    {
      if (controller.state != "active")
        continue;
      for (const auto& reset_controller : controller_names_)
      {
        if (controller.name == reset_controller)
          active.push_back(controller.name);
      }
    }
    return true;
  }

  bool SwitchControllers(const std::vector<std::string>& activate,
                         const std::vector<std::string>& deactivate,
                         std::string& error)
  {
    if (activate.empty() && deactivate.empty())
      return true;

    using namespace std::chrono_literals;
    if (!switch_client_->wait_for_service(1s))
    {
      error = "controller switch service is unavailable";
      return false;
    }

    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 3;

    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready)
    {
      error = "controller switch request timed out";
      return false;
    }
    const auto result = future.get();
    if (!result || !result->ok)
    {
      error = "controller manager rejected the switch";
      return false;
    }
    return true;
  }

  bool ApplyReset(ignition::gazebo::EntityComponentManager& ecm,
                  std::string& error)
  {
    for (std::size_t index = 0; index < names_.size(); ++index)
    {
      const auto entity = model_.JointByName(ecm, names_[index]);
      if (entity == ignition::gazebo::kNullEntity)
      {
        error = "unknown Gazebo joint: " + names_[index];
        return false;
      }
      ignition::gazebo::Joint joint(entity);
      joint.EnablePositionCheck(ecm, true);
      joint.EnableVelocityCheck(ecm, true);
      joint.ResetPosition(ecm, {positions_[index]});
      joint.ResetVelocity(ecm, {0.0});
    }
    return true;
  }

  void ClearLinkVelocities(ignition::gazebo::EntityComponentManager& ecm)
  {
    for (const auto entity : model_.Links(ecm))
    {
      ignition::gazebo::Link link(entity);
      link.SetLinearVelocity(ecm, ignition::math::Vector3d::Zero);
      link.SetAngularVelocity(ecm, ignition::math::Vector3d::Zero);
    }
  }

  bool ResetMatches(ignition::gazebo::EntityComponentManager& ecm,
                    std::string& error)
  {
    for (std::size_t index = 0; index < names_.size(); ++index)
    {
      const auto entity = model_.JointByName(ecm, names_[index]);
      if (entity == ignition::gazebo::kNullEntity)
      {
        error = "unknown Gazebo joint during reset verification: " + names_[index];
        return false;
      }
      ignition::gazebo::Joint joint(entity);
      const auto position = joint.Position(ecm);
      const auto velocity = joint.Velocity(ecm);
      if (!position || position->empty() || !velocity || velocity->empty())
      {
        error = "Gazebo joint state unavailable during reset verification: " + names_[index];
        return false;
      }
      if (std::abs(position->front() - positions_[index]) >
          reset_apply_tolerance_)
      {
        error = "Gazebo did not apply reset position for joint: " + names_[index];
        return false;
      }
      if (std::abs(velocity->front()) > reset_apply_tolerance_)
      {
        error = "Gazebo joint velocity is not zero after reset: " + names_[index];
        return false;
      }
    }
    return true;
  }

  void CompleteReset(bool success, std::string message)
  {
    completion_success_ = success;
    completion_message_ = std::move(message);
    completed_ = true;
    phase_ = ResetPhase::IDLE;
    condition_.notify_all();
  }

  ignition::gazebo::Model model_{ ignition::gazebo::kNullEntity };
  ignition::gazebo::EventManager* event_manager_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<ResetJoints>::SharedPtr service_;
  rclcpp::Service<ResetJoints>::SharedPtr mirror_service_;
  rclcpp::CallbackGroup::SharedPtr switch_callback_group_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::mutex service_mutex_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<std::string> names_;
  std::vector<double> positions_;
  std::vector<std::string> controller_names_;
  ResetPhase phase_{ResetPhase::IDLE};
  std::chrono::steady_clock::duration unpaused_start_sim_time_{};
  bool unpaused_time_started_{false};
  bool completed_{ false };
  bool completion_success_{ false };
  bool reset_applied_{ false };
  bool pause_world_{true};
  std::string completion_message_;
  std::string reset_apply_error_;
  double reset_apply_tolerance_{0.001};
  double reset_settle_duration_s_{0.1};
  double reset_verify_timeout_s_{1.0};
};
}  // namespace robot_arm3

IGNITION_ADD_PLUGIN(robot_arm3::JointResetSystem,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)
