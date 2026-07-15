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

#include "moveit_behavior/get_planning_scene.hpp"
namespace moveit_behavior
{
bool GetPlanningScene::setRequest(Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus GetPlanningScene::onResponseReceived(const Response::SharedPtr& response)
{
  auto planning_scene = response.get()->scene;

  if (!planning_scene.name.empty())
  {
    setOutput("planning_scene", planning_scene);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;

}

BT::NodeStatus GetPlanningScene::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  // namespace moveit_behavior
