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

#include "moveit_behavior/modify_collisions.hpp"
namespace moveit_behavior
{
bool ModifyCollisions::setRequest(Request::SharedPtr& request)
{
  moveit_msgs::msg::PlanningScene planning_scene;
    if (!getInput("planning_scene", planning_scene))
  {
    throw BT::RuntimeError("Could not access required blackboard input [planning_scene]");
  }

  std::vector<std::string> links_1;
  if (!getInput("links_1", links_1))
  {
    throw BT::RuntimeError("Could not access required blackboard input [links_1]");
  }
  std::vector<std::string> links_2;
  if (!getInput("links_2", links_2))
  {
    throw BT::RuntimeError("Could not access required blackboard input [links_2]");
  }
  bool disable_collisions;
  if (!getInput("disable_collisions", disable_collisions))
  {
    throw BT::RuntimeError("Could not access required blackboard input [disable_collisions]");
  }

  // Get current allowed collision matrix to check if links already exist in it
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg = planning_scene.allowed_collision_matrix;
  collision_detection::AllowedCollisionMatrix raw_acm(acm_msg);

  for (const auto& link1 : links_1) {
      for (const auto& link2 : links_2) {
          raw_acm.setEntry(link1, link2, disable_collisions);
      }
  }

  moveit_msgs::msg::AllowedCollisionMatrix modified_acm; 
  raw_acm.getMessage(modified_acm);

  // Apply the updated ACM back to the planning scene
  planning_scene.allowed_collision_matrix = modified_acm;
  planning_scene.is_diff = true;
  request->scene = planning_scene;

  return true;
}

BT::NodeStatus ModifyCollisions::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success)
  {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ModifyCollisions::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  // namespace moveit_behavior
