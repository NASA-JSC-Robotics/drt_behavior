#pragma once

#include "behaviortree_cpp/behavior_tree.h"

class PlanJointStates : public BT::SyncActionNode
{
public:
  PlanJointStates(const std::string& name, const BT::NodeConfig& config);

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts();

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
};