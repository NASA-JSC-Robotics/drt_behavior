#pragma once
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace moveit_behavior
{

    using SrvApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
    /**
 * @brief Modifies the Allowable Collision Matrix to enable/disable collisions between a set of links.
 *
 * @details Will apply the specified disable/enable collision bool to the all pairs of links provided for target_part.
 *
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | planning_scene             | Input     | moveit_msgs::msg::PlanningScene    |
 * | target_part                | Input     | std::vector<std::string>           |
 * | allow_collision            | Input     | bool                               |
 * | base_link                  |Input      | std::vector<std::string>           |
 */
class ModifyCollisions : public BT::RosServiceNode<SrvApplyPlanningScene>
{
public:
  explicit ModifyCollisions(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<SrvApplyPlanningScene>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        {   BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "planning scene from GetPlanningScene service"),
            BT::InputPort<std::vector<std::string> >("base_link", "First vector of objects to modify collisions for"),
            BT::InputPort<std::vector<std::string> >("target_part", "Target objects to modify collisions aganist base_link"),
            BT::InputPort<bool>("allow_collision", "True to disable collision checking, False to re-enable/enable it") });
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

// private:
// //   // Persistent client prevents undefined behavior and segmentation faults during async callbacks [1]
// //   std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
};

};