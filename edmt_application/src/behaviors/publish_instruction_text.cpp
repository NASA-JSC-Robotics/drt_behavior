#include "edmt_application/behaviors/publish_instruction_text.hpp"
#include "edmt_application/edmt_application.hpp"

PublishInstructionText::PublishInstructionText(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishInstructionText::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

    // input params
    BT::InputPort<std::string>("prompt", "Press next to continue.", "default is 'Press next to continue.'"),
    BT::InputPort<bool>("blocking", true, "Whether or not this waits for approval. default is true.'"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishInstructionText::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::string prompt;
  bool blocking;
  if (!getInput("prompt", prompt))
  {
    throw BT::RuntimeError("Could not access blackboard input [prompt]");
  }
  if (!getInput("blocking", blocking))
  {
    throw BT::RuntimeError("Could not access blackboard input [blocking]");
  }

  if (blocking)
  {
    edmt_application_node_->publish_instruction_text(prompt);
  }
  else
  {
    edmt_application_node_->publish_instruction_text_nb(prompt);
  }

  return BT::NodeStatus::SUCCESS;
}