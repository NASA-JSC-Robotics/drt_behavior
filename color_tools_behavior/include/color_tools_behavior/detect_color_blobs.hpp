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

#include <spdlog/spdlog.h>

#include <color_blob_centroid/color_blob_centroid.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

namespace color_tools_behavior
{
/**
 * @brief Given a synchronized set of images and camera info, identify color blobs
 * as specified by the user.
 */
class DetectColorBlobs : public BT::ThreadedAction
{
public:
  DetectColorBlobs(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  color_blob_centroid::BlobRequest blob_request;
};
}  // namespace color_tools_behavior
