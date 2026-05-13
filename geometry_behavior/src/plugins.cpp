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

#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_behavior/align_transform.hpp"
#include "geometry_behavior/apply_transform.hpp"
#include "geometry_behavior/async_tf_lookup.hpp"
#include "geometry_behavior/create_random_transform.hpp"
#include "geometry_behavior/publish_transform.hpp"
#include "geometry_behavior/transform_to_pose.hpp"
#include "geometry_behavior/pose_to_transform.hpp"

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<geometry_behavior::AlignTransform>("AlignTransform");
  factory.registerNodeType<geometry_behavior::ApplyTransform>("ApplyTransform");
  factory.registerNodeType<geometry_behavior::AsyncTfLookup>("AsyncTfLookup");
  factory.registerNodeType<geometry_behavior::TransformToPose>("TransformToPose");
  factory.registerNodeType<geometry_behavior::PoseToTransform>("PoseToTransform");
  factory.registerNodeType<geometry_behavior::CreateRandomTransform>("CreateRandomTransform");
  factory.registerNodeType<geometry_behavior::PublishTransform>("PublishTransform");
}
