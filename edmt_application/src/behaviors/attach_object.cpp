#include "edmt_application/behaviors/attach_object.hpp"
#include "edmt_application/edmt_application.hpp"

AttachObject::AttachObject(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList AttachObject::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

           // input params
           BT::InputPort<std::string>("move_group", std::string{ "" }, "default is empty string"),
           BT::InputPort<std::string>("object", std::string{ "" }, "default is empty string"),
           BT::InputPort<std::string>("link", std::string{ "" }, "default is empty string"),
           BT::InputPort<std::vector<std::string>>("touch_links", std::vector<std::string>{}, "default is empty vector")
  };
}

// You must override the virtual function tick()
BT::NodeStatus AttachObject::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::string move_group, object, link;
  std::vector<std::string> touch_links;
  if (!getInput("move_group", move_group))
  {
    throw BT::RuntimeError("Could not access blackboard input [move_group]");
  }
  if (!getInput("object", object))
  {
    throw BT::RuntimeError("Could not access blackboard input [object]");
  }
  if (!getInput("link", link))
  {
    throw BT::RuntimeError("Could not access blackboard input [link]");
  }
  if (!getInput("touch_links", touch_links))
  {
    throw BT::RuntimeError("Could not access blackboard input [touch_links]");
  }

  edmt_application_node_->set_move_group(move_group);
  auto result = edmt_application_node_->move_group_->attachObject(object, link, touch_links);

  if (result)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}