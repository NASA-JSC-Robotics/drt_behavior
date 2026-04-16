// behavior trees
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/bt_utils.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
// local
#include "drt_behavior/behaviors/check_param.hpp"

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

std::optional<std::string> DRTBehaviorBtcppExecutor::onTreeExecutionCompleted(BT::NodeStatus /*status*/,
                                                                              bool /*was_cancelled*/)
{
  // Clear the context, which should remove references. The blackboard will be deconstructed next run,
  // but remaining pointers should be useless
  if (context_)
  {
    context_->teardown(executor_);
    context_.reset();
  }

  running_ = false;
  return std::nullopt;
}

std::optional<BT::NodeStatus> DRTBehaviorBtcppExecutor::onLoopAfterTick(BT::NodeStatus /*status*/)
{ return std::nullopt; }
