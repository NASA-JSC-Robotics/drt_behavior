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

#include <tf2_eigen/tf2_eigen.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

#include "drt_behavior/drt_tree_context.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace geometry_behavior
{
/**
 * @brief Behavior to do a TF lookup based on a tf name
 *
 */
class WrenchCheck : public BT::StatefulActionNode
{
public:
  std::string topic_name;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber;

  geometry_msgs::msg::WrenchStamped::SharedPtr last_msg;

  std::vector<Eigen::Vector3d> forces;
  int queue_size;
  double threshold;
  WrenchCheck(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  void subCB(geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

};  // namespace geometry_behavior
