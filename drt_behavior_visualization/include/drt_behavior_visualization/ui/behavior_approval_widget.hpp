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

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
// BT
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/srv/get_trees.hpp"
// RVIZ2
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QString>
#include <QTimer>
#include <QtWidgets>
// STL
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */

#include <ui_approval_widget.h>

namespace drt_behavior_visualization
{
using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

class BehaviorApprovalWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BehaviorApprovalWidget(QWidget* parent);
  ~BehaviorApprovalWidget() = default;

private:
  std::unique_ptr<Ui::ApprovalWidget> ui_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr approval_service;
  QDialogButtonBox* approval_choice_widget;

  QTimer* timer;

  std::optional<rmw_request_id_t> request_header = std::nullopt;

  void timer_cb();

  void process_input(const bool& input);

private Q_SLOTS:
};

}  // namespace drt_behavior_visualization
