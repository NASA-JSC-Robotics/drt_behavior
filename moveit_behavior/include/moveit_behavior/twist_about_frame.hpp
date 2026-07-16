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

#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "drt_behavior/drt_tree_context.hpp"

#include <dex_ivr_helpers/dex_ivr_helpers.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

namespace moveit_behavior
{
/**
 * @brief Constructs motion planning requests and uses move_group's server client interface to find trajectory to a pose.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | group_name                 | Input     | std::string                        |
 * | trajectory                 | Output    | moveit_msgs::msg::RobotTrajectory  |
 */

/**
 * @brief Given a synchronized set of images and camera info, identify color blobs
 * as specified by the user.
 */
class TwistAboutFrame : public BT::ThreadedAction
{
public:
  TwistAboutFrame(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

};
}  // namespace moveit_behavior
