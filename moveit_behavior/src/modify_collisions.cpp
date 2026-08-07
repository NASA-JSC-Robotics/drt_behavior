#include "moveit_behavior/modify_collisions.hpp"

namespace moveit_behavior
{
bool ModifyCollisions::setRequest(Request::SharedPtr& request)
{
    moveit_msgs::msg::PlanningScene planning_scene;
    if(!getInput("planning_scene", planning_scene))
    {
        throw BT::RuntimeError("Could not access required blackboard input [planning_scene]");
    }

  std::vector<std::string> base_link;
  if (!getInput("base_link", base_link))
  {
    RCLCPP_WARN(logger(), "No blackboard input for base_link. Using default gripper links.");
    //default gripper links
    base_link = {"finger_1_link", "finger_2_link", "gripper_base_link"};
  }

  std::vector<std::string> target_part;
  if (!getInput("target_part", target_part))
  {
    throw BT::RuntimeError("Could not access required blackboard input [target_part]");
  }

  bool allow_collision;
  if (!getInput("allow_collision", allow_collision))
  {
    throw BT::RuntimeError("Could not access required blackboard input [allow_collision]");
  }

  // gets current allowed collision matrix to check if links exist in it
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg = planning_scene.allowed_collision_matrix;
  collision_detection::AllowedCollisionMatrix raw_acm(acm_msg);

  // updates acm 
  for (const auto& baselink : base_link)
  {
    for (const auto& targetpart : target_part)
    {
      raw_acm.setEntry(baselink, targetpart, allow_collision);
    }
  }

  moveit_msgs::msg::AllowedCollisionMatrix modified_acm;
  raw_acm.getMessage(modified_acm);

  // applies the updates acm back to the planning scene
  planning_scene.allowed_collision_matrix = modified_acm;
  planning_scene.is_diff = true;
  request->scene = planning_scene;

  return true;
}

BT::NodeStatus ModifyCollisions::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success)
  {
    RCLCPP_INFO(logger(), "ACM updated successfully!");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(logger(), "Failed to modify collisions.");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus ModifyCollisions::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Service call failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace moveit_behavior