#include "drt_behavior/behaviors/publish_trajectory.hpp"
#include "drt_behavior/drt_behavior.hpp"

PublishTrajectory::PublishTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishTrajectory::providedPorts()
{
  return {
    // global param
    BT::InputPort<std::shared_ptr<DRTBehavior>>("drt_behavior_node", "{@drt_behavior_node}"),

    // input params
    BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishTrajectory::tick()
{
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  if (!getInput("drt_behavior_node", drt_behavior_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [drt_behavior_node]");
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!getInput("trajectory", trajectory))
  {
    throw BT::RuntimeError("Could not access blackboard input [trajectory]");
  }

  drt_behavior_node_->visual_tools_->publishTrajectoryLine(
      trajectory, drt_behavior_node_->move_group_->getCurrentState()->getJointModelGroup(
                      drt_behavior_node_->active_planning_group));
  drt_behavior_node_->visual_tools_->trigger();

  return BT::NodeStatus::SUCCESS;
}