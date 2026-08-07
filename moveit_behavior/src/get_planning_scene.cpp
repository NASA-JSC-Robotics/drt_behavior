#include "moveit_behavior/get_planning_scene.hpp"

namespace moveit_behavior
{

bool GetPlanningScene::setRequest(Request::SharedPtr& request)
{ return true; }

BT::NodeStatus GetPlanningScene::onResponseReceived(const Response::SharedPtr& response)
{
  auto planning_scene = response.get()->scene;

  if (!planning_scene.name.empty())

{
    setOutput("planning_scene", planning_scene);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Error: Planning scene empty or missing components");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GetPlanningScene::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  