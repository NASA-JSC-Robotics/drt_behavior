#include "drt_behavior/behaviors/remove_collision_objects.hpp"
#include "drt_behavior/drt_behavior.hpp"

RemoveCollisionObjects::RemoveCollisionObjects(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList RemoveCollisionObjects::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

           // input params
           BT::InputPort<std::vector<std::string>>("objects", std::vector<std::string>({}), "defaults to empty vector")
  };
}

// You must override the virtual function tick()
BT::NodeStatus RemoveCollisionObjects::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }

  std::vector<std::string> objects;
  if (!getInput("objects", objects))
  {
    throw BT::RuntimeError("Could not access blackboard input [objects]");
  }

  drt_behavior_node_->planning_scene_interface_->removeCollisionObjects(objects);

  return BT::NodeStatus::SUCCESS;
}