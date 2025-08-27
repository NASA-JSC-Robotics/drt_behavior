#pragma once

#include "behaviortree_cpp/behavior_tree.h"

/**
 * @brief Behavior to plan a relative move based on a pose name
 *
 */
class PlanRelativeMove : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for CreateCollisionObject behavior
   *
   * @param name Name of behavior
   * @param config BTCPP node config
   */
  PlanRelativeMove(const std::string& name, const BT::NodeConfig& config);

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