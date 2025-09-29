#include "edmt_application/behaviors/remove_collision_objects.hpp"
#include "edmt_application/edmt_application.hpp"

RemoveCollisionObjects::RemoveCollisionObjects(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList RemoveCollisionObjects::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

           // input params
           BT::InputPort<std::vector<std::string>>("objects", std::vector<std::string>({}), "defaults to empty vector")
  };
}

// You must override the virtual function tick()
BT::NodeStatus RemoveCollisionObjects::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::vector<std::string> objects;
  if (!getInput("objects", objects))
  {
    throw BT::RuntimeError("Could not access blackboard input [objects]");
  }

  edmt_application_node_->planning_scene_interface_->removeCollisionObjects(objects);

  return BT::NodeStatus::SUCCESS;
}