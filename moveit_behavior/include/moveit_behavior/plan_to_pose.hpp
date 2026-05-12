
#pragma once
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace moveit_behavior
{
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
/**
 * @brief Constructs motion planning requests and uses move_group's server client interface to find trajectory to a pose.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | service_name               | Input     | std::string                        |
 * | group_name                 | Input     | std::string                        |
 * | end_effector_name          | Input     | std::string                        |
 * | position_tolerance         | Input     | double                             |
 * | orientation_tolerance      | Input     | std::vector<double>                |
 * | goal_pose                  | Input     | geometry_msgs::msg::PoseStamped    |
 * | velocity_scaling           | Input     | double                             |
 * | acceleration_scaling       | Input     | double                             |
 * | planner                    | Input     | std::string                        |
 * | pipeline                   | Input     | std::string                        |
 * | trajectory                 | Output    | moveit_msgs::msg::RobotTrajectory  |
 */
class PlanToPose : public BT::RosServiceNode<GetMotionPlan>
{
public:
  explicit PlanToPose(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<GetMotionPlan>(name, conf, params)
  {
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>("group_name", "the name of the planning group"),
          BT::InputPort<std::string>("end_effector_name", "the names of the end effector"),
          BT::InputPort<double>("position_tolerance", 0.01, "Raidus of sphere in which end effector position is valid"),
          BT::InputPort<std::vector<double> >("orientation_tolerance"),
          BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose",
                                                         "the positions of the joints (in same order as joint names)"),
          BT::InputPort<double>("velocity_scaling", 1.0, "type of moveit planner"),
          BT::InputPort<double>("acceleration_scaling", 1.0, "type of pipeline"),
          BT::InputPort<std::string>("planner", "RRTstarkConfigDefault", "type of moveit planner"),
          BT::InputPort<std::string>("pipeline", "ompl", "type of pipeline"),
          BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory") });
  }
  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};
};  // namespace moveit_behavior
