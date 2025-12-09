#include "drt_behavior/behaviors/plan_named_state.hpp"
#include "drt_behavior/drt_behavior.hpp"

PlanNamedState::PlanNamedState(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PlanNamedState::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

           // input params
           BT::InputPort<std::string>("move_group", std::string{ "none" }, "move_group to use for the plan"),
           BT::InputPort<std::string>("state_name", std::string{ "none" }, "name of the state to use for the plan"),
           BT::InputPort<float>("speed_scale", float{ 1.0 }, "default value is 1.0"),
           // output params
           BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory")
  };
}

// You must override the virtual function tick()
BT::NodeStatus PlanNamedState::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }
  std::string move_group, state_name;
  float speed_scale;
  if (!getInput("move_group", move_group))
  {
    throw BT::RuntimeError("Could not access blackboard input [move_group]");
  }
  if (!getInput("state_name", state_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [state_name]");
  }
  if (!getInput("speed_scale", speed_scale))
  {
    throw BT::RuntimeError("Could not access blackboard input [speed_scale]");
  }

  auto trajectory = drt_behavior_node_->plan_named_state(move_group, state_name, speed_scale);

  if (trajectory.has_value())
  {
    setOutput("trajectory", trajectory.value());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}