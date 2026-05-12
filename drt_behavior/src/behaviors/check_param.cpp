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

#include "drt_behavior/behaviors/check_param.hpp"

CheckParam::CheckParam(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList CheckParam::providedPorts()
{
  return { // input params
           BT::InputPort<std::string>("param_name")
  };
}

BT::NodeStatus CheckParam::onStart()
{
  if (!getInput("param_name", param_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [param_name]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  bool param_value = false;
  context->node->get_parameter_or(param_name, param_value, false);
  if (param_value)
  {
    context->node->set_parameter(rclcpp::Parameter(param_name, false));
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus CheckParam::onRunning()
{
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  bool param_value = false;
  context->node->get_parameter_or(param_name, param_value, false);
  if (param_value)
  {
    // context->node->undeclare_parameter(param_name);
    context->node->set_parameter(rclcpp::Parameter(param_name, false));
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void CheckParam::onHalted()
{ std::cout << "Interrupted [CheckParam] BT node" << std::endl; }
