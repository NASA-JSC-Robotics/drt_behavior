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

#include "geometry_behavior/transform_to_pose.hpp"

namespace geometry_behavior
{
TransformToPose::TransformToPose(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList TransformToPose::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")
  };
}

// You must override the virtual function tick()
BT::NodeStatus TransformToPose::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("transform_stamped", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [transform_stamped]");
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = t_stamped.header;
  pose_stamped.pose.position.x = t_stamped.transform.translation.x;
  pose_stamped.pose.position.y = t_stamped.transform.translation.y;
  pose_stamped.pose.position.z = t_stamped.transform.translation.z;

  pose_stamped.pose.orientation = t_stamped.transform.rotation;

  setOutput("pose_stamped", pose_stamped);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
