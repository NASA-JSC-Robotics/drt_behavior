

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

#include "drt_behavior_visualization/ui/behavior_approval_widget.hpp"

namespace drt_behavior_visualization
{
using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

BehaviorApprovalWidget::BehaviorApprovalWidget(QWidget* parent, rclcpp::Node::SharedPtr node_ptr)
  : QWidget(parent), node_(node_ptr), ui_(std::make_unique<Ui::ApprovalWidget>())
{
  std::cout << "Created a behavior approval widget" << std::endl;
  ui_->setupUi(this);

  approval_service = node_->create_service<std_srvs::srv::SetBool>(
      "add_two_ints", std::bind(&BehaviorApprovalWidget::toggle_continuous, this, _1, _2));

  //   // Extend the widget with all attributes and children from UI file
  //   ui_->setupUi(this);

  //   initialize_ui();

  //   get_behavior_trees_client_ = node_->create_client<btcpp_ros2_interfaces::srv::GetTrees>("/get_loaded_trees");

  //   stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/drt_behavior_stop");

  //   bt_status_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
  //       "/bt_action_server/bt_status", 10,
  //       std::bind(&BehaviorTreeWidget::bt_status_callback, this, std::placeholders::_1));

  //   execute_tree_client_ = rclcpp_action::create_client<ExecuteTree>(node_, "/bt_execution");

  //   // Set the initialized status
  //   // Right when we finish constructing the widget let actually send the request to get behavior trees
  //   on_UpdateTreesBtn_pressed();
}

}  // namespace drt_behavior_visualization
