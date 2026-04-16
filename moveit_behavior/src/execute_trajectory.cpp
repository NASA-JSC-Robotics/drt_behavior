#include "moveit_behavior/execute_trajectory.hpp"

namespace moveit_behavior
{
bool ExecuteTrajectory::setGoal(Goal& goal)
{
  if (!getInput("trajectory", goal.trajectory))
  {
    throw BT::RuntimeError("Could not access global blackboard input [trajectory]");
    return false;
  }
  return true;
}

BT::NodeStatus ExecuteTrajectory::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus ExecuteTrajectory::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace moveit_behavior
