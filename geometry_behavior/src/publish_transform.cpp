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

#include "geometry_behavior/publish_transform.hpp"

namespace geometry_behavior
{

PublishTransform::PublishTransform(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PublishTransform::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("input_transform"),
           BT::InputPort<std::string>("new_transform_name")
  };
}

// You must override the virtual function tick()
BT::NodeStatus PublishTransform::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("input_transform", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [input_transform]");
  }

  std::string new_transform_name;
  if (getInput("new_transform_name", new_transform_name))
  {
    t_stamped.child_frame_id = new_transform_name;
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  tf2_ros::TransformBroadcaster tf_broadcaster(context->node);
  tf_broadcaster.sendTransform(t_stamped);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace geometry_behavior
