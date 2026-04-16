#include "geometry_behavior/publish_transform.hpp"

namespace geometry_behavior
{

PublishTransform::PublishTransform(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishTransform::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("input_transform"),
           BT::InputPort<std::string>("new_transform_name")
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishTransform::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("input_transform", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [input_transform]");
  }

  std::string new_transform_name;
  if (getInput("new_transform_name", new_transform_name))
  {
    t_stamped.child_frame_id = new_transform_name;
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  tf2_ros::TransformBroadcaster tf_broadcaster(context->node);
  tf_broadcaster.sendTransform(t_stamped);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace geometry_behavior
