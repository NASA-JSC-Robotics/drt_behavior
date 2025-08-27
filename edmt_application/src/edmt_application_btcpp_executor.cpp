// behavior trees / ROS
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_executor_parameters.hpp"
#include "behaviortree_ros2/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"

// messages
#include "std_srvs/srv/set_bool.hpp"

// behaviors
#include "edmt_application/behaviors/create_collision_object.hpp"
#include "edmt_application/behaviors/plan_joint_states.hpp"
#include "edmt_application/behaviors/plan_relative_move.hpp"
#include "edmt_application/behaviors/prompt_and_execute.hpp"
#include "edmt_application/behaviors/tf_lookup.hpp"

// local
#include "edmt_application/edmt_application_btcpp_executor.hpp"

EdmtApplicationBtcppExecutor::EdmtApplicationBtcppExecutor(const rclcpp::NodeOptions& options)
  : BT::TreeExecutionServer(options)
{
  list_trees_service = node()->create_service<std_srvs::srv::SetBool>(
      "~/list_behavior_trees",
      std::bind(&EdmtApplicationBtcppExecutor::get_behavior_trees, this, std::placeholders::_1, std::placeholders::_2));
}

void EdmtApplicationBtcppExecutor::onTreeCreated(BT::Tree& tree)
{
  // logger_cout_ = std::make_shared<EdmtApplicationBtcppLogger>(tree);
  logger_cout_ = std::make_shared<EdmtApplicationBtcppLogger>(tree);

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
  globalBlackboard()->set("edmt_application_node", edmt_application_node_);
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

void EdmtApplicationBtcppExecutor::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  // initialize ROS node parameters
  BT::RosNodeParams params;
  // set ROS node
  params.nh = node();  // register with TreeExecutionServer's node

  factory.registerNodeType<TfLookup>("TfLookup");
  factory.registerNodeType<CreateCollisionObject>("CreateCollisionObject");
  factory.registerNodeType<PlanJointStates>("PlanJointStates");
  factory.registerNodeType<PlanRelativeMove>("PlanRelativeMove");
  factory.registerNodeType<PromptAndExecute>("PromptAndExecute");

  return;
}

void EdmtApplicationBtcppExecutor::get_behavior_trees(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto param_listener = std::make_shared<bt_server::ParamListener>(node());
  auto params = param_listener->get_params();
  // executeRegistration();
  factory().clearRegisteredBehaviorTrees();
  RegisterBehaviorTrees(params, factory(), node());

  response->success = true;

  auto tree_names = factory().registeredBehaviorTrees();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_logger"), "Here");
  std::string trees = "";
  for (const auto& name : tree_names)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("test_logger"), "Available tree: " << name << std::endl);
    trees += name + "\n";
  }
  response->message = trees;
}
