#pragma once

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

// using namespace std::chrono;

namespace moveit_behavior
{
using ExecTraj = moveit_msgs::action::ExecuteTrajectory;

class ExecuteTrajectory : public BT::RosActionNode<ExecTraj>
{
public:
  ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
    : BT::RosActionNode<ExecTraj>(name, config, params)
  {
  }

  static BT::PortsList providedPorts()
  { return providedBasicPorts({ BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory") }); };

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
};  // namespace moveit_behavior
