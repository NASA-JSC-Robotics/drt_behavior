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

#include "moveit_behavior/plan_to_joint_state.hpp"
namespace moveit_behavior
{
bool PlanToJointState::setRequest(Request::SharedPtr& request)
{
  // get planning inputs
  std::string group_name;
  if (!getInput("group_name", group_name))
  {
    throw BT::RuntimeError("Could not access required blackboard input [group_name]");
  }
  std::vector<std::string> joint_names;
  if (!getInput("joint_names", joint_names))
  {
    throw BT::RuntimeError("Could not access required blackboard input [joint_names]");
  }
  std::vector<double> joint_positions;
  if (!getInput("joint_positions", joint_positions))
  {
    throw BT::RuntimeError("Could not access required blackboard input [joint_positions]");
  }
  double tolerance;
  getInput("tolerance", tolerance);

  // verify same number of joint names and positions
  if (joint_names.size() != joint_positions.size())
  {
    throw BT::RuntimeError("Number of [joint_names] and [joint_positions] are not the same");
  }

  // create target joint state message
  target_js_.name.clear();
  target_js_.position.clear();

  moveit_msgs::msg::Constraints constraints;
  for (size_t i = 0; i < joint_names.size(); i++)
  {
    target_js_.name.push_back(joint_names[i]);
    target_js_.position.push_back(joint_positions[i]);

    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = joint_names[i];
    jc.position = joint_positions[i];
    jc.tolerance_above = tolerance;
    jc.tolerance_below = tolerance;
    jc.weight = 1.0;

    constraints.joint_constraints.push_back(jc);
  }

  request->motion_plan_request.group_name = group_name;
  request->motion_plan_request.goal_constraints.push_back(constraints);

  return true;
}

BT::NodeStatus PlanToJointState::onResponseReceived(const Response::SharedPtr& response)
{
  moveit_msgs::msg::MotionPlanResponse mpr = response.get()->motion_plan_response;
  if (mpr.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    setOutput("trajectory", mpr.trajectory);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus PlanToJointState::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  // namespace moveit_behavior
