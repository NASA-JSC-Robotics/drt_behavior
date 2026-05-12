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

#include "color_tools_behavior/color_blob_find.hpp"

namespace color_tools_behavior
{

bool ColorBlobFind::setRequest(Request::SharedPtr& request)
{
  getInput("desired_blob", request->desired_blob);
  getInput("min_blob_size", request->min_blob_size);
  getInput("color", request->color);

  return true;
}

BT::NodeStatus ColorBlobFind::onResponseReceived(const Response::SharedPtr& response)
{
  // I dont know how to properly check if the response is success full, so right now im just assuming that the
  // timestamp would 0 if its not found.
  geometry_msgs::msg::PoseStamped res = response.get()->centroid_pose;
  if (res.header.stamp.sec == 0 && res.header.stamp.nanosec == 0)
  {
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    setOutput("centroid_pose", res);
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus ColorBlobFind::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace color_tools_behavior
