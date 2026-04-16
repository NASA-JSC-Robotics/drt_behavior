#pragma once

#include <tf2_eigen/tf2_eigen.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "drt_behavior/drt_tree_context.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace geometry_behavior
{

/**
 * @brief Behavior to do a TF lookup based on a tf name
 *
 */
class PublishTransform : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for TfLookup behavior
   *
   * @param name Name of behavior
   * @param config BTCPP node config
   */
  PublishTransform(const std::string& name, const BT::NodeConfig& config);

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

};  // namespace geometry_behavior
