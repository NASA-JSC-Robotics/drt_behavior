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

#include "geometry_behavior/pose_to_transform.hpp"

namespace geometry_behavior
{
PoseToTransform::PoseToTransform(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PoseToTransform::providedPorts()
{
  return {
    // input parameters
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"),
    // output params
    BT::OutputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
  };
}

// You must override the virtual function tick()
BT::NodeStatus PoseToTransform::tick()
{
  geometry_msgs::msg::PoseStamped p_stamped;
  if (!getInput("pose_stamped", p_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [pose_stamped]");
  }

  geometry_msgs::msg::TransformStamped t_stamped;
  t_stamped.header = p_stamped.header;

  t_stamped.transform.translation.x = p_stamped.pose.position.x;
  t_stamped.transform.translation.y = p_stamped.pose.position.y;
  t_stamped.transform.translation.z = p_stamped.pose.position.z;

  t_stamped.transform.rotation = p_stamped.pose.orientation;

  setOutput("transform_stamped", t_stamped);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
