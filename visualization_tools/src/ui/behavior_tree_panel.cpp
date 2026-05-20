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

#include "visualization_tools/ui/behavior_tree_panel.hpp"

namespace visualization_tools
{
BehaviorTreePanel::BehaviorTreePanel(QWidget* parent)
  : rviz_common::Panel{ parent }, ui_(std::make_unique<Ui::rviz_panel>())
{
  // Extend the widget with all attributes and children from UI file
  ui_->setupUi(this);
}
// Overridden from Panel
void BehaviorTreePanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  abstract_node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // as per normal rclcpp code
  node_ = abstract_node_ptr_->get_raw_node();
  // Add the BehaviorTreeWidget to the GUI
  behavior_tree_widget_ = std::make_shared<BehaviorTreeWidget>(ui_->guiArea, node_);
  // ui_->guiArea = behavior_tree_widget_.get();
  ui_->guiArea->layout()->addWidget(behavior_tree_widget_.get());
  ui_->retranslateUi(this);
}
}  // namespace visualization_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(visualization_tools::BehaviorTreePanel, rviz_common::Panel)
