#include "edmt_application/behaviors/publish_trajectory.hpp"
#include "edmt_application/edmt_application.hpp"

PublishTrajectory::PublishTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishTrajectory::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

    // input params
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishTrajectory::tick()
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

  edmt_application_node_->visual_tools_->publishTrajectoryLine(
      trajectory, edmt_application_node_->move_group_->getCurrentState()->getJointModelGroup(
                      edmt_application_node_->active_planning_group));
  edmt_application_node_->visual_tools_->trigger();

  return BT::NodeStatus::SUCCESS;
}