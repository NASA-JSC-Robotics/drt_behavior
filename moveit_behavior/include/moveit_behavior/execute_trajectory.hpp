/* Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>


#include "std_msgs/msg/string.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace moveit_behavior
{
using ExecTraj = moveit_msgs::action::ExecuteTrajectory;
/**
 * @brief Execute given moveit robot trajectory using move_group's action client interface.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | action_name                | Input     | std::string                        |
 * | trajectory                 | Input     | moveit_msgs::msg::RobotTrajectory  |
 */
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

  void halt() override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
};  // namespace moveit_behavior
