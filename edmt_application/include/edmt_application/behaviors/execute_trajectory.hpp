#pragma once

#include "behaviortree_cpp/behavior_tree.h"

/**
 * @brief Behavior to prompt for a users approval from RViz before executing trajectory
 *
 */
class ExecuteTrajectory : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for ExecuteTrajectory behavior
   *
   * @param name Name of behavior
   * @param config BTCPP node config
   */
  ExecuteTrajectory(const std::string& name, const BT::NodeConfig& config);

  /**
   * @brief define provided ports for the behavior
   *
   * @return BT::PortsList Ports that the behavior will use
   */
  static BT::PortsList providedPorts();

  /**
   * @brief The portion of the node that actually does the meaningful work
   *
   * @return * BT::NodeStatus Status of the node
   */
  BT::NodeStatus tick() override;
};