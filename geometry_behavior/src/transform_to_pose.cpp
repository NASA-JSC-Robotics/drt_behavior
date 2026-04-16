#include "geometry_behavior/transform_to_pose.hpp"

namespace geometry_behavior
{
TransformToPose::TransformToPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList TransformToPose::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")
  };
}

// You must override the virtual function tick()
BT::NodeStatus TransformToPose::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("transform_stamped", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [transform_stamped]");
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = t_stamped.header;
  pose_stamped.pose.position.x = t_stamped.transform.translation.x;
  pose_stamped.pose.position.y = t_stamped.transform.translation.y;
  pose_stamped.pose.position.z = t_stamped.transform.translation.z;

  pose_stamped.pose.orientation = t_stamped.transform.rotation;

  setOutput("pose_stamped", pose_stamped);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
