#pragma once

#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "std_msgs/msg/string.hpp"

class DRTBehaviorBtcppLogger : public BT::StatusChangeLogger
{
public:
  DRTBehaviorBtcppLogger(const BT::Tree& tree, std::shared_ptr<rclcpp::Node> node);
  ~DRTBehaviorBtcppLogger() override;

  virtual void flush() override;

  /**
   * @brief struct to keep track of the status of a node and how to log it
   *
   */
  struct NodeLogStatus
  {
    NodeLogStatus(std::string name, BT::NodeStatus status, int indent) : name(name), status(status), indent(indent)
    {
    }
    NodeLogStatus() : name(""), status(BT::NodeStatus::IDLE), indent(0)
    {
    }
    std::string name;
    BT::NodeStatus status;
    int indent;
  };

  /**
   * @brief Callback that gets run on every node change
   *
   * @param timestamp Time of the node change
   * @param node The node object that just changed states
   * @param prev_status The previous node status
   * @param status The new node status
   */
  virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                        BT::NodeStatus status) override;

private:
  /**
   * @brief Method that generates the tree to use (run only at tree creation)
   *
   * @param node the node that will recursively search for children under
   * @param indent the amount that the node should be indented
   */
  void generateTree(const BT::TreeNode* node, int indent = 0);

  std::unordered_map<uint16_t, NodeLogStatus> log_statuses_;
  std::vector<uint16_t> log_order_;
  bool done_ = false;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_publisher_;
};
