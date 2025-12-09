#pragma once

// behavior trees / ROS
#include <drt_behavior/drt_behavior_btcpp_logger.hpp>
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

// messages
#include "drt_behavior_msgs/srv/get_behavior_trees.hpp"

// application node
#include "drt_behavior/drt_behavior.hpp"

class DRTBehaviorBtcppExecutor : public BT::TreeExecutionServer
{
public:
  /**
   * @brief Construct a new DRT Application Btcpp Executor object
   *
   * @param options Standard ros node options
   */
  DRTBehaviorBtcppExecutor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
   * @brief Callback for the service call to get list of the available behavior trees
   *
   * @param request Request of the setbool service
   * @param response Response of the setbool service
   */
  void get_behavior_trees(const std::shared_ptr<drt_behavior_msgs::srv::GetBehaviorTrees::Request> request,
                          std::shared_ptr<drt_behavior_msgs::srv::GetBehaviorTrees::Response> response);

  rclcpp::Service<drt_behavior_msgs::srv::GetBehaviorTrees>::SharedPtr list_trees_service;
  std::shared_ptr<DRTBehavior> drt_behavior_node_;
  std::shared_ptr<DRTBehaviorBtcppLogger> logger_cout_;
};