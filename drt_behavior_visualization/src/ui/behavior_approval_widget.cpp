

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

BehaviorApprovalWidget::BehaviorApprovalWidget(QWidget* parent)
  : QWidget(parent), ui_(std::make_unique<Ui::ApprovalWidget>())
{
  // Create a hidden node
  node_ = std::make_shared<rclcpp::Node>("_behavior_approval_node");
  // Setup ui
  ui_->setupUi(this);
  // Empty callback function, mandatory for service construction
  auto callback = [](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> /*response*/) {};
  approval_service = node_->create_service<std_srvs::srv::Trigger>("approval_service", callback);

  // Getting the widget by name, make it easier to use
  approval_choice_widget = parent->findChild<QDialogButtonBox*>("approvalChoice");
  // Keep the widget space even when hiding it.
  QSizePolicy sp_retain = approval_choice_widget->sizePolicy();
  sp_retain.setRetainSizeWhenHidden(true);
  approval_choice_widget->setSizePolicy(sp_retain);
  // Hide the widget
  approval_choice_widget->hide();

  // Connect widget's buttons to callbacks
  connect(approval_choice_widget, &QDialogButtonBox::accepted, [&]() { process_input(true); });
  connect(approval_choice_widget, &QDialogButtonBox::rejected, [&]() { process_input(false); });

  // Setup a timer
  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &BehaviorApprovalWidget::timer_cb);
  timer->start(100);  // 0.1 seconds
}

void BehaviorApprovalWidget::timer_cb()
{
  // Instead of having RViz executor spin the node and trigger the callback when a request is sent,
  // we check if there is a request manually
  // If there is a request we show the widget and wait for use to make their choice
  std_srvs::srv::Trigger::Request request;
  rmw_request_id_t req_id;
  if (approval_service->take_request(request, req_id))
  {
    request_header = req_id;
    approval_choice_widget->show();
  }
}

void BehaviorApprovalWidget::process_input(const bool& input)
{
  // Check if there optional variable is set
  if (request_header)
  {
    // return user's response
    std_srvs::srv::Trigger::Response response;
    response.success = input;
    approval_service->send_response(request_header.value(), response);
    request_header.reset();
  }
  // Hide the widget
  approval_choice_widget->hide();
}

}  // namespace drt_behavior_visualization
