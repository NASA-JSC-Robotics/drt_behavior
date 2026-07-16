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

#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"

namespace moveit_behavior
{
using SrvGetPlanningScene = moveit_msgs::srv::GetPlanningScene;
/**
 * @brief Gets planning scene from available service.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | planning_scene             | Output    | moveit_msgs::msg::PlanningScene    |
 */
class GetPlanningScene : public BT::RosServiceNode<SrvGetPlanningScene>
{
public:
  explicit GetPlanningScene(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<SrvGetPlanningScene>(name, conf, params)
  {
  }
  static BT::PortsList providedPorts()
  { return providedBasicPorts({ BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene") }); }
  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
};  // namespace moveit_behavior
