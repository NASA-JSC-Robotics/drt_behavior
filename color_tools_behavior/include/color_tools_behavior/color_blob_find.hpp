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
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "color_tools_msgs/srv/blob_centroid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace color_tools_behavior
{
using ClrBlbFind = color_tools_msgs::srv::BlobCentroid;

class ColorBlobFind : public BT::RosServiceNode<ClrBlbFind>
{
public:
  explicit ColorBlobFind(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
    : BT::RosServiceNode<ClrBlbFind>(name, config, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<int>("desired_blob", 0, "Desired Blob index. Default is 0"),
          BT::InputPort<double>("min_blob_size", 10.0, "Minimum size of width or height of object in pixels"),
          BT::InputPort<std::string>("color", "red", "color of the mask to use"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>("centroid_pose", "Centroid of the found blob") });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};

};  // namespace color_tools_behavior
