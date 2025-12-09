#include "drt_behavior/behaviors/prompt_and_execute.hpp"
#include "drt_behavior/drt_behavior.hpp"

PromptAndExecute::PromptAndExecute(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PromptAndExecute::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

    // input params
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
    BT::InputPort<std::string>("prompt", "Press next to continue."),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PromptAndExecute::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
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

  auto result = drt_behavior_node_->prompt_and_execute(trajectory, prompt);

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