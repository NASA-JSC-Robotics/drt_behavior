#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "drt_behavior/drt_tree_context.hpp"

class CheckParam : public BT::StatefulActionNode
{
public:
  std::string param_name;
  /**
   * @brief Constructor for TfLookup behavior
   *
   * @param name Name of behavior
   * @param config BTCPP node config
   */
  CheckParam(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief define provided ports for the behavior
   *
   * @return BT::PortsList Ports that the behavior will use
   */
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};
