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
#include "drt_behavior/drt_tree_context.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace geometry_behavior
{

/**
 * @brief Behavior to do a TF lookup based on a tf name
 *
 */
class PublishTransform : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for TfLookup behavior
   *
   * @param name Name of behavior
   * @param config BTCPP node config
   */
  PublishTransform(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief define provided ports for the behavior
   *
   * @return BT::PortsList Ports that the behavior will use
   */
  static BT::PortsList providedPorts();

  /**
   * @brief The portion of the node that actually does the meaningful work
   *
   * @return * BT::NodeStatus Status of the node
   */
  BT::NodeStatus tick() override;
};

};  // namespace geometry_behavior
