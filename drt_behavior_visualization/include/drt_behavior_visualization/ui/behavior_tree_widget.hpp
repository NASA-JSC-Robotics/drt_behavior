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
#include "std_srvs/srv/trigger.hpp"
// BT
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/srv/get_trees.hpp"
// RVIZ2
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QString>
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
#include <ui_behavior_tree_widget.h>

namespace drt_behavior_visualization
{
using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

class BehaviorTreeWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BehaviorTreeWidget(QWidget* parent, rclcpp::Node::SharedPtr node_ptr);

  ~BehaviorTreeWidget() = default;

  // #region PRIVATE

private:
  // #tag GUI
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Ui::behavior_tree_widget> ui_;

  const std::string green = "\033[92m";
  const std::string red = "\033[91m";
  const std::string blue = "\033[96m";
  const std::string yellow = "\033[93m";
  const std::string end_color = "\033[0m";

  std::string active_behavior = "";

  std::vector<std::string> available_behaviors;

  // #tag Class_Members
  // Widget Initialized
  bool widget_initialized_ = false;

  std::vector<std::string> log;
  std::vector<std::string> bt_text;

  rclcpp_action::Client<btcpp_ros2_interfaces::action::ExecuteTree>::SharedPtr execute_tree_client_;

  rclcpp::Client<btcpp_ros2_interfaces::srv::GetTrees>::SharedPtr get_behavior_trees_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr logger_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_status_subscriber_;

  // methods
  void initialize_ui();
  // callbacks

  void result_callback(const GoalHandleExecuteTree::WrappedResult& result);
  void goal_response_callback(const GoalHandleExecuteTree::SharedPtr& goal_handle);

  void get_behavior_trees_cb(const rclcpp::Client<btcpp_ros2_interfaces::srv::GetTrees>::SharedFuture future);

  void logger_callback(const std_msgs::msg::String::SharedPtr msg);
  void bt_status_callback(const std_msgs::msg::String::SharedPtr msg);

  std::string replace_string(std::string string_to_replace, std::string old_text, std::string new_text);

  // Update panel data
  void update_bt_text();
  void update_log_text();
  std::string process_text(std::string text);

  void waitForClient(rclcpp::ClientBase::SharedPtr client);

private Q_SLOTS:
  void on_StopButton_pressed();
  void on_UpdateTreesBtn_pressed();
  void on_RunButton_pressed();
};

}  // namespace drt_behavior_visualization
