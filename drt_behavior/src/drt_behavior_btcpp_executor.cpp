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

// behavior trees
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/bt_utils.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
// local
#include "drt_behavior/behaviors/add_user_log.hpp"
#include "drt_behavior/behaviors/call_trigger_service.hpp"
#include "drt_behavior/behaviors/check_param.hpp"
#include "drt_behavior/behaviors/publish_string.hpp"

#include "drt_behavior/drt_behavior_btcpp_executor.hpp"

using namespace drt_behavior;

// Executor is passed into the constructor to have it spin the ROS node created for the use of behavior trees.
DRTBehaviorBtcppExecutor::DRTBehaviorBtcppExecutor(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor,
                                                   const rclcpp::NodeOptions& options)
  : BT::TreeExecutionServer(options), executor_(executor), running_(false)
{ globalBlackboard()->set<std::shared_ptr<DRTTreeContext>>("drt_context", nullptr); }

void DRTBehaviorBtcppExecutor::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<CheckParam>("CheckParam");
  factory.registerNodeType<AddUserLog>("AddUserLog");

  BT::RosNodeParams call_trigger_params(node());
  call_trigger_params.server_timeout = std::chrono::milliseconds(600 * 1000);

  factory.registerNodeType<CallTriggerService>("CallTriggerService", call_trigger_params);
  factory.registerNodeType<PublishString>("PublishString", call_trigger_params);

  return;
}

bool DRTBehaviorBtcppExecutor::onGoalReceived(const std::string& /*tree_name*/, const std::string& /*payload*/)
{
  if (running_)
  {
    RCLCPP_WARN(node()->get_logger(), "Rejecting goal: a tree is already executing");
    return false;
  }
  else
  {
    context_ = DRTTreeContext::create(executor_);
    globalBlackboard()->set<std::shared_ptr<DRTTreeContext>>("drt_context", context_);
  }
  return true;
}

void DRTBehaviorBtcppExecutor::onTreeCreated(BT::Tree& tree)
{
  // Create logger with the new tree create
  logger_ptr = std::make_shared<DRTBehaviorBtcppLogger>(tree, node());
  // Start the thing running...
  running_ = true;
}

std::optional<std::string> DRTBehaviorBtcppExecutor::onTreeExecutionCompleted(BT::NodeStatus status,
                                                                              bool /*was_cancelled*/)
{
  std::stringstream combined_user_log;
  // Clear the context, which should remove references. The blackboard will be deconstructed next run,
  // but remaining pointers should be useless
  if (context_)
  {
    if (context_->user_logs.size())
    {
      combined_user_log << "Tree finished with status: " << BT::toStr(status);
      for (auto user_log : context_->user_logs)
      {
        combined_user_log << "\n" << user_log;
      }
    }

    context_->teardown(executor_);
    context_.reset();
  }

  running_ = false;

  if (combined_user_log.str().size())
  {
    return combined_user_log.str();
  }

  return std::nullopt;
}

std::optional<BT::NodeStatus> DRTBehaviorBtcppExecutor::onLoopAfterTick(BT::NodeStatus status)
{
  // If a tick fails we kill the tree
  if (status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_ERROR(node()->get_logger(), "Tree returned FAILURE, stopping execution");
    return status;
  }
  // otherwise continue
  return std::nullopt;
}
