#include "behaviortree_cpp/bt_factory.h"
#include "moveit_behavior/execute_trajectory.hpp"
#include "moveit_behavior/plan_to_joint_state.hpp"
#include "moveit_behavior/plan_to_pose.hpp"
#include "moveit_behavior/publish_trajectory.hpp"

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<moveit_behavior::ExecuteTrajectory>("ExecuteTrajectory", params);
  factory.registerNodeType<moveit_behavior::PlanToJointState>("PlanToJointState", params);
  factory.registerNodeType<moveit_behavior::PlanToPose>("PlanToPose", params);
  factory.registerNodeType<moveit_behavior::PublishDisplayTrajectory>("PublishDisplayTrajectory", params);
}
