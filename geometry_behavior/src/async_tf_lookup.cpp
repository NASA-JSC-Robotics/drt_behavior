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

#include "geometry_behavior/async_tf_lookup.hpp"

namespace geometry_behavior
{

AsyncTfLookup::AsyncTfLookup(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList AsyncTfLookup::providedPorts()
{
  return { // input params
           BT::InputPort<double>("lookup_timeout", 1.0, "N/A"),
           BT::InputPort<std::string>("base_frame"), BT::InputPort<std::string>("target_frame"),
           // output params
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("tf")
  };
}
BT::NodeStatus AsyncTfLookup::onStart()
{
  getInput<double>("lookup_timeout", lookup_timeout);


  if (!getInput("base_frame", base_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [base_frame]");
  }
  if (!getInput("target_frame", target_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  try
  {
    auto result = context->tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
    setOutput("tf", result);
    start_time = std::chrono::steady_clock::now();
    return BT::NodeStatus::SUCCESS;
  }
  catch (const tf2::TransformException& ex)
  {
    start_time = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus AsyncTfLookup::onRunning()
{
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  if (elapsed < lookup_timeout)
  {
    auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
    try
    {
      auto result = context->tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
      setOutput("tf", result);
      RCLCPP_DEBUG(context->node->get_logger(), "Found transform : %s -> %s", base_frame.c_str(), target_frame.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_DEBUG(context->node->get_logger(), "Looking for transform : %s -> %s", base_frame.c_str(),
                  target_frame.c_str());
      return BT::NodeStatus::RUNNING;
    }
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void AsyncTfLookup::onHalted()
{ std::cout << "Interrupted [AsyncTfLookup] BT node" << std::endl; }

}  // namespace geometry_behavior
