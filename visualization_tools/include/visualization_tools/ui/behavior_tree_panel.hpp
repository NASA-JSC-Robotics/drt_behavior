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

#pragma once

// STL
#include <chrono>
#include <memory>
#include <string>
// ROS2
#include <rclcpp/rclcpp.hpp>
// RVIZ2
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QGroupBox>
#include <QScrollArea>
#include <QTabWidget>
#include <QtWidgets>

#include <ui_rviz2_panel.h>
#include <visualization_tools/ui/behavior_tree_widget.hpp>

namespace visualization_tools
{
class BehaviorTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget* parent = nullptr);
  void onInitialize() override;

protected:
  // ROS Stuff
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> abstract_node_ptr_;

  // Widgets
  std::shared_ptr<BehaviorTreeWidget> behavior_tree_widget_;

private:
  std::unique_ptr<Ui::rviz_panel> ui_;
  // private Q_SLOTS:
};
};  // namespace visualization_tools
