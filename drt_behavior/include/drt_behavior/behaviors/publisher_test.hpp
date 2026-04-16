#pragma once

#include "behaviortree_cpp/behavior_tree.h"

#include "drt_behavior/drt_tree_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherTest : public BT::SyncActionNode
{
public:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  PublisherTest(const std::string& name, const BT::NodeConfig& config);
  ~PublisherTest();

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
