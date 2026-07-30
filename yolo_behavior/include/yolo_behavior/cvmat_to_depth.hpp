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

#include <sensor_msgs/msg/image.hpp>

#include "color_tools_msgs/srv/blob_centroid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "drt_behavior/drt_tree_context.hpp"

#include "ros2_yolos_cpp/conversion/pose_converter.hpp"
#include "ros2_yolos_cpp/nodes/pose_node.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace yolo_behavior
{
class CVMatToDepth : public BT::StatefulActionNode
{
public:
  CVMatToDepth(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

  void unsubscribeTopics();

private:
  std::weak_ptr<rclcpp::Node> node_;

  bool done = false;

  float conf_, nms_;

  void imageCB(const sensor_msgs::msg::Image::SharedPtr msg);

  std::string service_suffix_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher;

  static constexpr std::size_t kMessageSyncQueueSize = 10;
  static constexpr std::chrono::seconds kMessageSyncTimeout{ 5 };

  std::string debug_image_topic;
  bool publish_debug_image = false;

  std::vector<ros2_yolos_cpp::SegmentationResult> segmentation_result;

  static constexpr auto kSegmentationResult = "segmentation_result";
  static constexpr auto kDepthTopic = "depth_topic";
  static constexpr auto kDepthImage = "depth_image";
};

};  // namespace yolo_behavior
