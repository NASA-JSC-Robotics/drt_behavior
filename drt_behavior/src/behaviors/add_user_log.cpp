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

#include "drt_behavior/behaviors/add_user_log.hpp"

AddUserLog::AddUserLog(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList AddUserLog::providedPorts()
{
  return { // input params
           BT::InputPort<std::string>("user_log")
  };
}

BT::NodeStatus AddUserLog::tick()
{
  std::string user_log;
  if (!getInput("user_log", user_log))
  {
    throw BT::RuntimeError("Could not access blackboard input [user_log]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  context->user_logs.push_back(user_log);

  // bool param_value = false;
  // context->node->get_parameter_or(param_name, param_value, false);
  // if (param_value)
  // {
  //   context->node->set_parameter(rclcpp::Parameter(param_name, false));
  //   return BT::NodeStatus::SUCCESS;
  // }
  // else
  // {
  //   return BT::NodeStatus::RUNNING;
  // }

  // auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  // bool param_value = false;
  // context->node->get_parameter_or(param_name, param_value, false);
  // if (param_value)
  // {
  //   // context->node->undeclare_parameter(param_name);
  //   context->node->set_parameter(rclcpp::Parameter(param_name, false));
  //   return BT::NodeStatus::SUCCESS;
  // }
  // else
  // {
  //   return BT::NodeStatus::RUNNING;
  // }
  return BT::NodeStatus::SUCCESS;
}
