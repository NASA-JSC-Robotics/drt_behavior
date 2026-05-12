#include "moveit_behavior/publish_trajectory.hpp"
namespace moveit_behavior
{

bool PublishDisplayTrajectory::setMessage(moveit_msgs::msg::DisplayTrajectory& msg)
{
  if (!getInput("trajectory", msg.trajectory))
  {
    throw BT::RuntimeError("Could not access required blackboard input [trajectory]");
    return false;
  }
  return true;
}

}  // namespace moveit_behavior
