#include "drt_behavior/behaviors/tf_lookup.hpp"
#include "drt_behavior/drt_behavior.hpp"

TfLookup::TfLookup(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList TfLookup::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

           // input params
           BT::InputPort<std::string>("base_frame"), BT::InputPort<std::string>("target_frame"),
           // output params
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("tf")
  };
}

// You must override the virtual function tick()
BT::NodeStatus TfLookup::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }

  std::string base_frame, target_frame;
  if (!getInput("base_frame", base_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [base_frame]");
  }
  if (!getInput("target_frame", target_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }

  auto result = drt_behavior_node_->tf_lookup(base_frame, target_frame);

  setOutput("tf", result);
  return BT::NodeStatus::SUCCESS;
}