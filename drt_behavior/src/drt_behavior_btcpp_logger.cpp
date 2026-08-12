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

#include "drt_behavior/drt_behavior_btcpp_logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("DRT_btcpp_logger");
}

DRTBehaviorBtcppLogger::DRTBehaviorBtcppLogger(const BT::Tree& tree, std::shared_ptr<rclcpp::Node> node)
  : BT::StatusChangeLogger(tree.rootNode()), node_(node)
{
  rclcpp::QoS qos_profile(10);                                        // History depth of 10
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);       // Reliable delivery
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);  // Volatile durability

  // Create a publisher so that we can
  bt_status_publisher_ = node_->create_publisher<std_msgs::msg::String>("~/bt_status", qos_profile);

  generateTree(tree.rootNode());
}
DRTBehaviorBtcppLogger::~DRTBehaviorBtcppLogger()
{
}

void DRTBehaviorBtcppLogger::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node, BT::NodeStatus prev_status,
                                      BT::NodeStatus status)
{
  if (status == BT::NodeStatus::IDLE)
  {
    // we want to stay shown as success or failure, not show idle again, so leave it as is
    if (prev_status == BT::NodeStatus::FAILURE || prev_status == BT::NodeStatus::SUCCESS)
    {
      return;
    }
  }

  log_statuses_[node.UID()].status = status;

  std::stringstream logmsg_stream;
  // should be print tree function
  for (uint16_t id : log_order_)
  {
    std::string indent_string(log_statuses_[id].indent, ' ');
    logmsg_stream << (indent_string + log_statuses_[id].name + ": " + toStr(log_statuses_[id].status, true)) << ";";
  }

  std_msgs::msg::String msg;
  msg.data = logmsg_stream.str();
  bt_status_publisher_->publish(msg);
}

void DRTBehaviorBtcppLogger::generateTree(const BT::TreeNode* node, int indent)
{
  // on the first run, clear these data
  if (indent == 0)
  {
    log_statuses_.clear();
    log_order_.clear();
  }

  auto node_status = NodeLogStatus(node->name(), BT::NodeStatus::IDLE, indent);
  log_statuses_[node->UID()] = node_status;
  log_order_.push_back(node->UID());

  // iterate recursively through children to add them to the things
  if (auto control = dynamic_cast<const BT::ControlNode*>(node))
  {
    for (const auto& child : control->children())
    {
      generateTree(static_cast<const BT::TreeNode*>(child), indent + 1);
    }
  }
  else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
  {
    generateTree(decorator->child(), indent + 1);
  }
}

void DRTBehaviorBtcppLogger::flush()
{ std::cout << std::flush; }
