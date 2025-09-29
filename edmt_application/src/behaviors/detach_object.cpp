#include "edmt_application/behaviors/detach_object.hpp"
#include "edmt_application/edmt_application.hpp"

DetachObject::DetachObject(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList DetachObject::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

    // input params
    BT::InputPort<std::string>("move_group", std::string{ "" }, "default is empty string"),
    BT::InputPort<std::string>("object", std::string{ "" }, "default is empty string"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus DetachObject::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::string move_group, object;
  if (!getInput("move_group", move_group))
  {
    throw BT::RuntimeError("Could not access blackboard input [move_group]");
  }
  if (!getInput("object", object))
  {
    throw BT::RuntimeError("Could not access blackboard input [object]");
  }

  edmt_application_node_->set_move_group(move_group);
  auto result = edmt_application_node_->move_group_->detachObject(object);

  if (result)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}