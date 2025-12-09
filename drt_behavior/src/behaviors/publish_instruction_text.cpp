#include "drt_behavior/behaviors/publish_instruction_text.hpp"
#include "drt_behavior/drt_behavior.hpp"

PublishInstructionText::PublishInstructionText(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishInstructionText::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

    // input params
    BT::InputPort<std::string>("prompt", "Press next to continue.", "default is 'Press next to continue.'"),
    BT::InputPort<bool>("blocking", true, "Whether or not this waits for approval. default is true.'"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishInstructionText::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
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
    drt_behavior_node_->publish_instruction_text(prompt);
  }
  else
  {
    drt_behavior_node_->publish_instruction_text_nb(prompt);
  }

  return BT::NodeStatus::SUCCESS;
}