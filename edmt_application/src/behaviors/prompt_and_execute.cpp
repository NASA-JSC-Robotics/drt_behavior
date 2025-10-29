#include "edmt_application/behaviors/prompt_and_execute.hpp"
#include "edmt_application/edmt_application.hpp"

PromptAndExecute::PromptAndExecute(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PromptAndExecute::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

    // input params
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
    BT::InputPort<std::string>("prompt", "Press next to continue."),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PromptAndExecute::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  std::string prompt;
  if (!getInput("trajectory", trajectory))
  {
    throw BT::RuntimeError("Could not access blackboard input [trajectory]");
  }
  if (!getInput("prompt", prompt))
  {
    throw BT::RuntimeError("Could not access blackboard input [prompt]");
  }

  auto result = edmt_application_node_->prompt_and_execute(trajectory, prompt);

  if (result.has_value())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    throw BT::RuntimeError(result.error());
    return BT::NodeStatus::FAILURE;
  }
}