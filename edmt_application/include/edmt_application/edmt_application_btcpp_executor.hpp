#pragma once

// behavior trees / ROS
#include <edmt_application/edmt_application_btcpp_logger.hpp>
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

// messages
#include "std_srvs/srv/set_bool.hpp"

// application node
#include "edmt_application/edmt_application.hpp"

class EdmtApplicationBtcppExecutor : public BT::TreeExecutionServer
{
public:
  /**
   * @brief Construct a new Edmt Application Btcpp Executor object
   *
   * @param options Standard ros node options
   */
  EdmtApplicationBtcppExecutor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
  void get_behavior_trees(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr list_trees_service;
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  std::shared_ptr<EdmtApplicationBtcppLogger> logger_cout_;
};