#include "drt_behavior/behaviors/plan_joint_states.hpp"
#include "drt_behavior/drt_behavior.hpp"

PlanJointStates::PlanJointStates(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PlanJointStates::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

           // input params
           BT::InputPort<std::string>("joint_state_name"),
           BT::InputPort<float>("speed_scale", float{ 1.0 }, "default value is 1.0"),
           // output params
           BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory")
  };
}

// You must override the virtual function tick()
BT::NodeStatus PlanJointStates::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }
  std::string joint_state_name;
  float speed_scale;
  if (!getInput("joint_state_name", joint_state_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [joint_state_name]");
  }
  if (!getInput("speed_scale", speed_scale))
  {
    throw BT::RuntimeError("Could not access blackboard input [speed_scale]");
  }

  auto trajectory = drt_behavior_node_->plan_joint_states(joint_state_name, speed_scale);

  if (trajectory.has_value())
  {
    setOutput("trajectory", trajectory.value());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}