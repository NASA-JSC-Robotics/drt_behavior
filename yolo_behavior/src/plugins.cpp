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
#include "yolo_behavior/detect_pose.hpp"

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{ factory.registerNodeType<yolo_behavior::DetectPose>("DetectPose"); }
