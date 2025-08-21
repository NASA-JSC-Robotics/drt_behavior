#pragma once

// behavior trees / ROS
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/tree_execution_server.hpp"
#include <edmt_application/edmt_application_btcpp_logger.hpp>


// messages
#include "std_srvs/srv/set_bool.hpp"

// application node
#include "edmt_application/edmt_application.hpp"

class EdmtApplicationBtcppExecutor : public BT::TreeExecutionServer
{
public:
    EdmtApplicationBtcppExecutor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;

    void onTreeCreated(BT::Tree& tree) override;

    void get_behavior_trees(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);

    void statusChangeCallback(BT::TimePoint timestamp, const BT::TreeNode& btnode, BT::NodeStatus prev, BT::NodeStatus status);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr list_trees_service;
    std::shared_ptr<EdmtApplication> edmt_application_node_;
    std::shared_ptr<EdmtApplicationBtcppLogger> logger_cout_;
};