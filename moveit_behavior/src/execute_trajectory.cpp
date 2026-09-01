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

#include "moveit_behavior/execute_trajectory.hpp"

namespace moveit_behavior
{
bool ExecuteTrajectory::setGoal(Goal& goal)
{
  if (!getInput("trajectory", goal.trajectory))
  {
    throw BT::RuntimeError("Could not access global blackboard input [trajectory]");
    return false;
  }
  return true;
}

BT::NodeStatus ExecuteTrajectory::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void ExecuteTrajectory::halt()
{
  if (auto node = node_.lock())
  {
    auto pub = node->create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 1);
    auto message = std_msgs::msg::String();
    message.data = "stop";
    pub->publish(message);
    pub.reset();
  }
  BT::RosActionNode<ExecTraj>::halt();
}

BT::NodeStatus ExecuteTrajectory::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace moveit_behavior
