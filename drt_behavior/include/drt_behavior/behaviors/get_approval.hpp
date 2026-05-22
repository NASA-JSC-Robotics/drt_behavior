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

#include "std_srvs/srv/trigger.hpp"

namespace drt_behavior
{
using Trigger = std_srvs::srv::Trigger;
/**
 * @brief Constructs motion planning requests and uses move_group's server client interface to find trajectory to a joint state.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 */
class GetApproval : public BT::RosServiceNode<Trigger>
{
public:
  explicit GetApproval(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<Trigger>(name, conf, params)
  {
  }
  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};
};  // namespace drt_behavior
