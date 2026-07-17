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

#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"

namespace moveit_behavior
{
using SrvApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
/**
 * @brief Modifies the Allowable Collision Matrix  to enable/disable collisions between two lists of links.
 *
 * @details Will apply the specified disable/enable collision bool to the all pairs of links provided in both lists.
 *
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | planning_scene             | Input     | moveit_msgs::msg::PlanningScene    |
 * | links_1                    | Input     | std::vector<std::string>           |
 * | links_2                    | Input     | std::vector<std::string>           |
 * | disable_collisions         | Input     | bool                               |
 */
class ModifyCollisions : public BT::RosServiceNode<SrvApplyPlanningScene>
{
public:
  explicit ModifyCollisions(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<SrvApplyPlanningScene>(name, conf, params)
  {
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene",
                                                         "planning scene from the GetPlanningScene service"),
          BT::InputPort<std::vector<std::string> >("links_1", "first vector of objects to modify collisions for"),
          BT::InputPort<std::vector<std::string> >("links_2",
                                                   "second vector of objects to modify collisions against links_1"),
          BT::InputPort<bool>("disable_collisions", true,
                              "true to disable collisions between links, false to enable collisions") });
  }
  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
};  // namespace moveit_behavior
