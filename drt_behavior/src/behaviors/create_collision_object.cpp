#include "drt_behavior/behaviors/create_collision_object.hpp"
#include "drt_behavior/drt_behavior.hpp"

CreateCollisionObject::CreateCollisionObject(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList CreateCollisionObject::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

    // input params
    BT::InputPort<std::string>("collision_object_name"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus CreateCollisionObject::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }

  std::string collision_object_name;
  if (!getInput("collision_object_name", collision_object_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [collision_object_name]");
  }

  auto result = drt_behavior_node_->create_collision_object(collision_object_name);

  if (result.has_value())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_FATAL(rclcpp::get_logger("create_collision_object"), result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
}