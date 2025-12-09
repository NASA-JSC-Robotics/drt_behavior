// behavior trees / ROS
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"

// behaviors
#include "drt_behavior/behaviors/attach_object.hpp"
#include "drt_behavior/behaviors/create_collision_object.hpp"
#include "drt_behavior/behaviors/detach_object.hpp"
#include "drt_behavior/behaviors/execute_trajectory.hpp"
#include "drt_behavior/behaviors/plan_joint_states.hpp"
#include "drt_behavior/behaviors/plan_named_state.hpp"
#include "drt_behavior/behaviors/plan_relative_move.hpp"
#include "drt_behavior/behaviors/prompt_and_execute.hpp"
#include "drt_behavior/behaviors/publish_instruction_text.hpp"
#include "drt_behavior/behaviors/publish_trajectory.hpp"
#include "drt_behavior/behaviors/remove_collision_objects.hpp"
#include "drt_behavior/behaviors/tf_lookup.hpp"
#include "drt_behavior/behaviors/update_collision_matrix.hpp"

// local
#include "drt_behavior/drt_behavior_btcpp_executor.hpp"

DRTBehaviorBtcppExecutor::DRTBehaviorBtcppExecutor(const rclcpp::NodeOptions& options)
  : BT::TreeExecutionServer(options)
{
  list_trees_service = node()->create_service<drt_behavior_msgs::srv::GetBehaviorTrees>(
      "~/list_behavior_trees",
      std::bind(&DRTBehaviorBtcppExecutor::get_behavior_trees, this, std::placeholders::_1, std::placeholders::_2));
}

void DRTBehaviorBtcppExecutor::onTreeCreated(BT::Tree& tree)
{
  // logger_cout_ = std::make_shared<DRTBehaviorBtcppLogger>(tree);
  logger_cout_ = std::make_shared<DRTBehaviorBtcppLogger>(tree, node());

  //   std::map<int, std::string> UID_to_path;
  //     tree.applyVisitor([&UID_to_path](BT::TreeNode* node) {
  //     UID_to_path[node->UID()] = node->fullPath();
  //     std::cout << node->UID() << " -> " << node->fullPath() << std::endl;
  //   });
  // for (auto& subtree : tree.subtrees)
  // {
  //   for (auto& btnode : subtree->nodes)
  //   {
  //     std::string msg = btnode->fullPath();
  //     RCLCPP_INFO(node()->get_logger(), msg.c_str());
  //   }
  // }

  // first_timestamp_ = std::chrono::high_resolution_clock::now();

  // auto subscribeCallback = [this](TimePoint timestamp, const TreeNode& node,
  //                                 NodeStatus prev, NodeStatus status) {
  //     if(enabled_ && (status != NodeStatus::IDLE || show_transition_to_idle_))
  //     {
  //     if(type_ == TimestampType::absolute)
  //     {
  //         this->callback(timestamp.time_since_epoch(), node, prev, status);
  //     }
  //     else
  //     {
  //         this->callback(timestamp - first_timestamp_, node, prev, status);
  //     }
  //     }
  // };

  // auto visitor = [this, subscribeCallback](TreeNode* node) {
  //     subscribers_.push_back(node->subscribeToStatusChange(std::move(subscribeCallback)));
  // };

  // applyRecursiveVisitor(root_node, visitor);

  // put move group interface node on the blackboard
  globalBlackboard()->set("drt_behavior_node", drt_behavior_node_);
  // // put robot description and planning group on blackboard
  // globalBlackboard()->set("robot_description_topic", robot_description_topic_);
  // globalBlackboard()->set("default_planning_group", default_planning_group_);
  // globalBlackboard()->set("full_robot_planning_group", full_robot_planning_group_);
  // allows all nodes in the tree to access

  // note that move group cannot be placed on blackboard
  // because it is not copy-constructable
  // instead, each node will create its own move group instance

  return;
}

void DRTBehaviorBtcppExecutor::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  // initialize ROS node parameters
  BT::RosNodeParams params;
  // set ROS node
  params.nh = node();  // register with TreeExecutionServer's node

  factory.registerNodeType<AttachObject>("AttachObject");
  factory.registerNodeType<CreateCollisionObject>("CreateCollisionObject");
  factory.registerNodeType<DetachObject>("DetachObject");
  factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
  factory.registerNodeType<PlanJointStates>("PlanJointStates");
  factory.registerNodeType<PlanNamedState>("PlanNamedState");
  factory.registerNodeType<PlanRelativeMove>("PlanRelativeMove");
  factory.registerNodeType<PromptAndExecute>("PromptAndExecute");
  factory.registerNodeType<PublishInstructionText>("PublishInstructionText");
  factory.registerNodeType<PublishTrajectory>("PublishTrajectory");
  factory.registerNodeType<RemoveCollisionObjects>("RemoveCollisionObjects");
  factory.registerNodeType<TfLookup>("TfLookup");
  factory.registerNodeType<UpdateCollisionMatrix>("UpdateCollisionMatrix");

  return;
}

void DRTBehaviorBtcppExecutor::get_behavior_trees(
    const std::shared_ptr<drt_behavior_msgs::srv::GetBehaviorTrees::Request> /*request*/,
    std::shared_ptr<drt_behavior_msgs::srv::GetBehaviorTrees::Response> response)
{
  auto param_listener = std::make_shared<bt_server::ParamListener>(node());
  auto params = param_listener->get_params();
  // executeRegistration();
  factory().clearRegisteredBehaviorTrees();
  RegisterBehaviorTrees(params, factory(), node());

  auto tree_names = factory().registeredBehaviorTrees();
  std::vector<std::string> trees;
  for (const auto& name : tree_names)
  {
    trees.push_back(name);
  }
  RCLCPP_INFO(node()->get_logger(), "Responding with available behavior trees");
  response->behavior_trees = trees;

}
