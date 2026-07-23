#include "moveit_behavior/toggle_collision_action.hpp"

namespace moveit_behavior
{
bool ToggleCollisionAction::setRequest(Request::SharedPtr& request)
{
  bool allow;
  if (!getInput("allow_collision", allow))
  {
    throw BT::RuntimeError("Could not access required blackboard input [allow_collision]");
  }

  std::string target;
  if (!getInput("target_part", target))
  {
    throw BT::RuntimeError("Could not access required blackboard input [target_part]");
  }

  // Lock the internal node_ weak_ptr to get the shared_ptr instance
  auto rclcpp_node = node_.lock();
  if (!rclcpp_node)
  {
    throw BT::RuntimeError("The underlying ROS 2 node instance is no longer valid.");
  }

  // Instantiate the client as a persistent class member if not already done
  if (!param_client_)
  {
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(rclcpp_node, "/acm_modifier_node");
  }
  
  if (param_client_->service_is_ready())
  {
    // Capture a copy of the logger instance and the node_ weak_ptr
    auto logger_instance = logger();
    auto weak_node_ref = node_;

    param_client_->set_parameters({rclcpp::Parameter("target_part", target)},
      [weak_node_ref, logger_instance](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        // Ensure the underlying ROS node still exists prior to running callback logic
        auto node_shared = weak_node_ref.lock();
        if (!node_shared) {
          return; 
        }

        try {
          auto results = future.get();
          if (!results.empty() && !results[0].successful) {
            RCLCPP_ERROR(logger_instance, "acm_modifier_node rejected the target_part parameter change: %s", 
                         results[0].reason.c_str());
          } else {
            RCLCPP_INFO(logger_instance, "Successfully updated target_part parameter asynchronously.");
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(logger_instance, "Failed to set parameter asynchronously: %s", e.what());
        }
      });
  }
  else
  {
    RCLCPP_WARN(logger(), "acm_modifier_node parameter server not ready. Proceeding with service request anyway.");
  }

  request->data = allow;
  return true;
}

BT::NodeStatus ToggleCollisionAction::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success)
  {
    RCLCPP_INFO(logger(), "ACM updated via service. Message: %s", response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(logger(), "Service failed updating ACM. Message: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus ToggleCollisionAction::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Service call failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  // namespace moveit_behavior