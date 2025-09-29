#include "edmt_application/behaviors/execute_trajectory.hpp"
#include "edmt_application/edmt_application.hpp"

ExecuteTrajectory::ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList ExecuteTrajectory::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

    // input params
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus ExecuteTrajectory::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!getInput("trajectory", trajectory))
  {
    throw BT::RuntimeError("Could not access blackboard input [trajectory]");
  }

  auto result = edmt_application_node_->execute_movement(trajectory);

  if (result.has_value())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    throw BT::RuntimeError(result.error());
    return BT::NodeStatus::SUCCESS;
  }
}