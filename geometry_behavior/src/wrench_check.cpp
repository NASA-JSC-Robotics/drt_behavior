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

#include "geometry_behavior/wrench_check.hpp"

namespace geometry_behavior
{

WrenchCheck::WrenchCheck(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
{
}

BT::PortsList WrenchCheck::providedPorts()
{
  return { // input params
           BT::InputPort<std::string>("topic_name"), //
           BT::InputPort<std::string>("queue_size"),
           BT::InputPort<double>("threshold"),
           BT::OutputPort<std::shared_ptr<geometry_msgs::msg::WrenchStamped>>("wrench_stamped")
  };
}

BT::NodeStatus WrenchCheck::onStart()
{
  if (!getInput("topic_name", topic_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [topic_name]");
  }

  if (!getInput("queue_size", queue_size))
  {
    throw BT::RuntimeError("Could not access blackboard input [queue_size]");
  }

  if (!getInput("threshold", threshold))
  {
    throw BT::RuntimeError("Could not access blackboard input [threshold]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  wrench_subscriber = context->node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_name, 1, std::bind(&WrenchCheck::subCB, this, std::placeholders::_1));
  
  forces.clear();
  return BT::NodeStatus::RUNNING;
}

void WrenchCheck::subCB(geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr)
{
  last_msg = msg_ptr;
  geometry_msgs::msg::WrenchStamped world_wrench_ft;
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  try{
    auto transform = context->tf_buffer->lookupTransform("world", last_msg->header.frame_id, tf2::TimePointZero);
    tf2::doTransform(*last_msg, world_wrench_ft, transform);
    setOutput("wrench_stamped", world_wrench_ft);
    Eigen::Vector3d force(world_wrench_ft.wrench.force.x, world_wrench_ft.wrench.force.y, world_wrench_ft.wrench.force.z);
    forces.push_back(force);
  }
  catch (const tf2::TransformException &ex) 
  {
    std::cout << "Failed to find the transform" << std::endl;
  }
  return;
}

BT::NodeStatus WrenchCheck::onRunning()
{
  if (forces.size() > queue_size)
  {
    forces.erase(forces.begin());

    // Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    double sum = 0.0;
    for (const auto& v : forces)
    {
      sum += v.norm();
    }
    // Eigen::Vector3d avg_vector = sum / forces.size();
    double avg_vector = sum / forces.size();
    std::cout << "AVERAGE VECTOR:\n" << avg_vector << std::endl; 
    setOutput("wrench_stamped", last_msg);

    if (avg_vector > threshold)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WrenchCheck::onHalted()
{ std::cout << "Interrupted [WrenchCheck] BT node" << std::endl; }

}  // namespace geometry_behavior
