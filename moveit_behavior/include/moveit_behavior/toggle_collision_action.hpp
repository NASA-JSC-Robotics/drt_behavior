#pragma once

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "std_srvs/srv/set_bool.hpp"

namespace moveit_behavior
{
using SetBoolSrv = std_srvs::srv::SetBool;

/**
 * @brief Interfaces with the custom acm_modifier_node to toggle MoveIt allowed collision matrices.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | allow_collision            | Input     | bool                               |
 * | target_part                | Input     | std::string                        |
 * 
 * *Note: 'service_name' is implicitly provided by BT::RosServiceNode base implementation.*
 */
class ToggleCollisionAction : public BT::RosServiceNode<SetBoolSrv>
{
public:
  ToggleCollisionAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosServiceNode<SetBoolSrv>(name, conf, params)
  {
  }

  // Mandatory metadata name matching macro for the dynamic system
  static const std::string& providedNodeName() 
  {
    static const std::string name = "ToggleCollision";
    return name;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<bool>("allow_collision", "True to disable collision checking, False to enable it"),
          BT::InputPort<std::string>("target_part", "The name of the target object/link to toggle") });
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  // Persistent client prevents undefined behavior and segmentation faults during async callbacks [1]
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
};

}  // namespace moveit_behavior