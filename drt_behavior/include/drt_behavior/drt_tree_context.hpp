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

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace drt_behavior
{

/**
 * @brief Per-execution ROS infrastructure for DRT Demos.
 */
struct DRTTreeContext
{
  /// The ROS node that all BT nodes should use for publishers, clients, etc.
  std::shared_ptr<rclcpp::Node> node;

  /// Shared TF buffer.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  /// Transform listener that feeds tf_buffer.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // User logs
  std::vector<std::string> user_logs;

  /**
   * @brief Factory function to create a fully-initialised context and add its node to the executor.
   * @param executor The executor to spin the relevant context items.
   * @param node_name Optional name for the per-execution node.
   */
  static std::shared_ptr<DRTTreeContext> create(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor,
                                                const std::string& node_name = "drt_behavior_tree_node");

  /**
   * @brief Tear down every resource and remove the node from the executor.
   * @details all resources held by the context should be released, so stale pointers
   * on the blackboard should be harmless (hopefully).
   * @param executor This should be the same executor that was passed during creation.
   */
  void teardown(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);
};

}  // namespace drt_behavior
