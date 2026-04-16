#pragma once

#include <atomic>

// behavior trees
#include "behaviortree_ros2/tree_execution_server.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"

#include "drt_behavior/drt_tree_context.hpp"

// application node
#include <drt_behavior/drt_behavior_btcpp_logger.hpp>

namespace drt_behavior
{
class DRTBehaviorBtcppExecutor : public BT::TreeExecutionServer
{
public:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  BT::Tree* current_tree;
  std::shared_ptr<DRTBehaviorBtcppLogger> logger_ptr;

  /**
   * @brief Construct a new DRT Application Btcpp Executor object
   *
   * @param options Standard ros node options
   */
  DRTBehaviorBtcppExecutor(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor,
                           const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool onGoalReceived(const std::string& tree_name, const std::string& payload) override;

  /**
   * @brief Registers BTCPP nodes into the factory (overrides base class)
   *
   * @param factory The factory that we register nodes into
   */
  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;

  /**
   * @brief Function that runs when the behavior tree is created
   *
   * @param tree Access to the tree that was just created
   */
  void onTreeCreated(BT::Tree& tree) override;

  /**
   * @brief onTreeExecutionCompleted is a callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
   *
   * @return if not std::nullopt, the string will be sent as [return_message] to the Action Client.
   */
  virtual std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;

  /**
   * @brief onLoopAfterTick invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
   */
  virtual std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

private:
  /// Shared context for each tree, this should be fresh and clean each run
  std::shared_ptr<DRTTreeContext> context_;

  /// Is a tree running?
  std::atomic_bool running_;
};

}  // namespace drt_behavior
