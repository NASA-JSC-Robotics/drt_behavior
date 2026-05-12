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
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace moveit_behavior
{
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
/**
 * @brief Constructs motion planning requests and uses move_group's server client interface to find trajectory to a joint state.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | group_name                 | Input     | std::string                        |
 * | joint_names                | Input     | std::vector<std::string>           |
 * | joint_positions            | Input     | std::vector<double>                |
 * | tolerance                  | Input     | double                             |
 * | trajectory                 | Output    | moveit_msgs::msg::RobotTrajectory  |
 */
class PlanToJointState : public BT::RosServiceNode<GetMotionPlan>
{
private:
  // planning parameters
  sensor_msgs::msg::JointState target_js_;
  // scaling parameters
  double velocity_scaling_ = 0.8;
  double acceleration_scaling_ = 0.8;

public:
  explicit PlanToJointState(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<GetMotionPlan>(name, conf, params)
  {
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>("group_name", "the name of the planning group"),
          BT::InputPort<std::vector<std::string>>("joint_names", "the names of the joints"),
          BT::InputPort<std::vector<double>>("joint_positions",
                                             "the positions of the joints (in same order as joint names)"),
          BT::InputPort<double>("tolerance", 0.0436332, "tolerance above and below for joint constraint"),
          BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory") });
  }
  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};
};  // namespace moveit_behavior
