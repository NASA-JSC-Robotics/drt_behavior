#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <drt_behavior/drt_behavior.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

auto const logger = rclcpp::get_logger("drt_behavior");

DRTBehavior::DRTBehavior(std::string default_planning_group, rclcpp::NodeOptions node_options)
  : Node("drt_behavior", node_options), active_planning_group(default_planning_group)
{
  logger_publisher_ = this->create_publisher<std_msgs::msg::String>("/application_logger", 10);

  get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  apply_planning_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  switch_controller_client_ =
      this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "drt_behavior_stop", std::bind(&DRTBehavior::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void DRTBehavior::initialize()
{
  // set_move_group(active_planning_group);
  // planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

  // transform lookup overhead
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  load_configs();
}

/*
Utilities
*/
void DRTBehavior::publish_instruction_text(std::string prompt)
{
  std::string instruction_text = prompt + blue + " Press next to continue." + end_color;

  std_msgs::msg::String msg;
  msg.data = instruction_text;
  logger_publisher_->publish(msg);

  visual_tools_->prompt(prompt + " Press next to continue.");
  publish_instruction_text_nb("Continuing...");
}

void DRTBehavior::publish_instruction_text_nb(std::string prompt)
{
  std::string instruction_text = prompt;

  std_msgs::msg::String msg;
  msg.data = instruction_text;
  logger_publisher_->publish(msg);

  RCLCPP_INFO(logger, "%s", prompt.c_str());
}

void DRTBehavior::make_behavior_tree()
{
  local_behavior_tree = "";
  printTree(tree_head);
  {
    std::lock_guard<std::mutex> lock(behavior_tree_mutex);
    behavior_tree = local_behavior_tree;
  }
}

void DRTBehavior::printTree(const std::shared_ptr<BehaviorItem>& item, const std::string& prefix, bool isLast)
{
  std::string color_string = "";
  if (item->status == BehaviorStatus::Active)
    color_string = blue;
  if (item->status == BehaviorStatus::Success)
    color_string = green;
  if (item->status == BehaviorStatus::Failure)
    color_string = red;
  std::string end_color_string = (item->status != BehaviorStatus::Pending) ? end_color : "";
  local_behavior_tree += prefix + (isLast ? "└── " : "├── ") + color_string + item->name + end_color_string + '\n';

  std::string newPrefix = prefix + (isLast ? "    " : "│   ");
  auto it = item->children.begin();
  auto end = item->children.end();
  for (auto i = it; i != end; ++i)
  {
    if (i + 1 == end)
    {
      printTree(*i, newPrefix, true);
    }
    else
    {
      printTree(*i, newPrefix, false);
    }
  }
}

bool DRTBehavior::load_configs()
{
  // --- Grab yaml parameters, and setup the Transform lookup
  std::string drt_behavior_config;
  this->get_parameter("drt_behavior_config", drt_behavior_config);
  RCLCPP_INFO(logger, "Loading YAML file of configs: %s", drt_behavior_config.c_str());
  config_yaml = YAML::LoadFile(drt_behavior_config);
  return true;
}

tl::expected<geometry_msgs::msg::Transform, std::string> DRTBehavior::get_tf_from_yaml(YAML::Node node)
{
  geometry_msgs::msg::Transform tf;

  tf.translation.x = node["tx"].as<double>();
  tf.translation.y = node["ty"].as<double>();
  tf.translation.z = node["tz"].as<double>();
  tf.rotation.x = node["qx"].as<double>();
  tf.rotation.y = node["qy"].as<double>();
  tf.rotation.z = node["qz"].as<double>();
  tf.rotation.w = node["qw"].as<double>();

  if (node["modifier"])
  {
    auto modifier_string = node["modifier"].as<std::string>();
    if (!config_yaml["modifiers"][modifier_string])
    {
      return tl::make_unexpected("Could not find the modifier " + modifier_string +
                                 " in the config `modifiers` section");
    }
    auto modifier_tf = get_tf_from_yaml(config_yaml["modifiers"][modifier_string]);
    auto tf_eigen = tf2::transformToEigen(tf);
    auto modifier_eigen = tf2::transformToEigen(modifier_tf.value());
    auto modified_tf = modifier_eigen * tf_eigen;
    tf = tf2::eigenToTransform(modified_tf).transform;
  }

  return tf;
}

/*
Flow Control
*/

void DRTBehavior::stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  cancel_behaviors = true;
  response->success = true;
}

std::pair<bool, std::string> DRTBehavior::call_behavior(std::string behavior_name)
{
  if (!function_registry.count(behavior_name))
  {
    std::string available_behaviors = "";
    for (auto const& behaviors : function_registry)
      available_behaviors += "\n\t" + behaviors.first;

    std::string error_message = red + "Behavior " + behavior_name +
                                " does not exist. Available behaviors are:" + available_behaviors + end_color;
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    return std::pair<bool, std::string>(false, error_message);
  }

  if (!current_node)
  {
    tree_head = std::make_shared<BehaviorItem>(behavior_name);
    current_node = tree_head;
  }
  else
  {
    current_node->children.push_back(std::make_shared<BehaviorItem>(behavior_name));
    current_node->children[current_node->children.size() - 1]->parent = current_node;
    current_node = current_node->children[current_node->children.size() - 1];
    current_node->parent->status = BehaviorStatus::Pending;
  }

  make_behavior_tree();

  auto result = std::pair<bool, std::string>(false, "default_message");

  if (cancel_behaviors)
  {
    cancel_behaviors = false;
    std::string error_message =
        red + "Behavior " + behavior_name + " was skipped because the STOP flag was set." + end_color;
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    result = std::pair<bool, std::string>(false, error_message);
  }
  else
  {
    std::string message = green + "Running behavior: " + behavior_name + end_color;
    RCLCPP_INFO(logger, "%s", message.c_str());
    result = function_registry[behavior_name]();
    if (!result.first)
    {
      std::string error_message = red + "Behavior " + behavior_name + " failed. Reason: " + result.second + end_color;
      result.second = error_message;
      RCLCPP_ERROR(logger, "%s", error_message.c_str());
    }
  }

  current_node->status = result.first ? BehaviorStatus::Success : BehaviorStatus::Failure;

  if (current_node->parent)
  {
    current_node = current_node->parent;
  }
  else
  {
    current_node = nullptr;
    make_behavior_tree();
  }
  return result;
};

/*
MoveIt and ROS Utilities
*/
void DRTBehavior::set_move_group(std::string move_group_name)
{
  active_planning_group = move_group_name;
  move_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), active_planning_group);

  move_group_->setPlannerId("RRTstarkConfigDefault");
  move_group_->setPlanningTime(1.0);

  move_group_->setNumPlanningAttempts(5);

  // --- Set up rviz visual tools, this is the GUI in the bottom left of RVIZ that lets you step through trajectories.
  namespace rvt = rviz_visual_tools;
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "base_link", "move_group_tutorial", move_group_->getRobotModel());
  visual_tools_->loadRemoteControl();
}

// look up a frame and convert it to a pose for cartesian move
geometry_msgs::msg::TransformStamped DRTBehavior::tf_lookup(std::string target_frame, std::string base_frame)
{
  // //Lookup Transform for offsets
  geometry_msgs::msg::TransformStamped tf;
  int counter = 0;
  bool found_buffer = false;
  while (counter++ < 100 && !found_buffer)
  {
    try
    {
      tf = tf_buffer_->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
      found_buffer = true;
    }
    catch (const std::exception& e)
    {
      std::chrono::nanoseconds wait_time(10'000'000);
      rclcpp::sleep_for(wait_time);
    }
  }
  if (!found_buffer)
  {
    RCLCPP_ERROR(logger, "DID NOT FIND TF FROM [%s] to [%s]", base_frame.c_str(), target_frame.c_str());
  }

  return tf;
}

geometry_msgs::msg::Pose DRTBehavior::tf_lookup_converted(std::string target_frame, std::string base_frame)
{
  auto tf = tf_lookup(target_frame, base_frame);
  geometry_msgs::msg::Pose waypoint;
  waypoint.position.x = tf.transform.translation.x;
  waypoint.position.y = tf.transform.translation.y;
  waypoint.position.z = tf.transform.translation.z;
  waypoint.orientation = tf.transform.rotation;

  return waypoint;
}

tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
DRTBehavior::plan_cartesian_waypoint_pose(geometry_msgs::msg::Pose waypoint, float speed_scale)
{
  // Assemble the cartesian waypoitns to move through (current position to waypoint)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::PoseStamped start_pose = move_group_->getCurrentPose();
  waypoints.push_back(start_pose.pose);
  waypoints.push_back(waypoint);

  // Compute the cartesian path to the waypoint
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.00;
  const double eef_step = 0.01;
  double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // SCALE the speed
  robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), active_planning_group);
  rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

  // Here, the mention that you cant do velocity and acceleration scaling with cartesian planning
  // https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html
  // and instead the reference the page below, which recommends to do manual velocity scaling, which follows
  // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
  // Third create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  iptp.computeTimeStamps(rt, speed_scale, speed_scale);
  rt.getRobotTrajectoryMsg(trajectory);
  // END scale the speed

  bool success = (1.0 - fraction) < 1e-3;
  RCLCPP_INFO(logger, "Cartesian path computation: %0.2f percent. %s", fraction * 100.0,
              success ? "SUCCESS!" : "FAILURE!");

  std::string error_msg;
  if (!success)
  {
    std::stringstream percent_formatted;
    percent_formatted << std::fixed << std::setprecision(2) << fraction * 100.0;
    return tl::make_unexpected("Cartesian path only computed " + percent_formatted.str() + "%% percent of the path");
  }
  return trajectory;
}

tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
DRTBehavior::plan_joint_waypoint_pose(geometry_msgs::msg::Pose waypoint, float speed_scale)
{
  move_group_->setMaxVelocityScalingFactor(speed_scale);
  move_group_->setMaxAccelerationScalingFactor(speed_scale);

  // Eigen::Isometry3d waypoint_eigen = tf2::transformToEigen(waypoint);
  // auto rsp = move_group_->getRobotStatePtr();
  // auto jmg = move_group_->getCurrentState()->getJointModelGroup(active_planning_group);
  // rsp->setFromIk(jmg, waypoint_eigen);
  // move_group_->setJointTarget(&rsp);

  move_group_->setPoseTarget(waypoint);

  moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan;
  auto success = (move_group_->plan(joint_space_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Joint space computation: %s", success ? "SUCCESS!" : "FAILED!");

  if (!success)
    return tl::make_unexpected("Joint space plan failed. See moveit terminal for error");
  return joint_space_plan.trajectory_;
}

tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
DRTBehavior::plan_relative_move(std::string relative_move_name, PlanType move_type, float speed_scale)
{
  set_move_group(config_yaml["relative_poses"][relative_move_name]["move_group"].as<std::string>());

  auto reference_frame = config_yaml["relative_poses"][relative_move_name]["reference"].as<std::string>();
  auto relative_tf = get_tf_from_yaml(config_yaml["relative_poses"][relative_move_name]);
  if (!relative_tf.has_value())
    return tl::make_unexpected(relative_tf.error());

  auto reference_tf = tf_lookup(reference_frame);
  auto reference_eig = tf2::transformToEigen(reference_tf);
  auto relative_eig = tf2::transformToEigen(relative_tf.value());

  auto final_pose = reference_eig * relative_eig;
  geometry_msgs::msg::Pose relative_pose = Eigen::toMsg(final_pose);

  auto target_tf = tf2::eigenToTransform(final_pose);
  target_tf.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  target_tf.header.frame_id = "world";
  target_tf.child_frame_id = "target_pose";
  tf_static_broadcaster_->sendTransform(target_tf);

  if (move_type == PlanType::Cartesian)
  {
    return plan_cartesian_waypoint_pose(relative_pose, speed_scale);
  }
  else
  {
    return plan_joint_waypoint_pose(relative_pose, speed_scale);
  }
}

tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
DRTBehavior::plan_joint_states(std::string joint_state_name, float speed_scale)
{
  set_move_group(config_yaml["joint_states"][joint_state_name]["move_group"].as<std::string>());

  std::vector<double> joint_group_positions;
  for (const auto& joint_data : config_yaml["joint_states"][joint_state_name]["positions"])
  {
    joint_group_positions.push_back(joint_data.as<double>());
  }

  move_group_->setJointValueTarget(joint_group_positions);
  move_group_->setMaxVelocityScalingFactor(speed_scale);
  move_group_->setMaxAccelerationScalingFactor(speed_scale);

  moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan;
  auto success = (move_group_->plan(joint_space_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Joint space computation: %s", success ? "SUCCESS!" : "FAILED!");

  if (!success)
    return tl::make_unexpected("Joint space plan failed. See moveit terminal for error");
  return joint_space_plan.trajectory_;
}

tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
DRTBehavior::plan_named_state(std::string move_group, std::string state_name, float speed_scale)
{
  set_move_group(move_group);
  move_group_->setMaxVelocityScalingFactor(speed_scale);
  move_group_->setMaxAccelerationScalingFactor(speed_scale);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group_->setNamedTarget(state_name);
  auto success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(logger, "Named state computation: %s", success ? "SUCCESS!" : "FAILED!");

  if (!success)
    return tl::make_unexpected("Named state plan failed. See moveit terminal for error");

  return my_plan.trajectory_;
}

tl::expected<void, std::string> DRTBehavior::prompt_and_execute(moveit_msgs::msg::RobotTrajectory trajectory,
                                                                std::string prompt)
{
  if (cancel_behaviors)
  {
    cancel_behaviors = false;
    return tl::make_unexpected("Not planning the next move because the STOP flag was set.");
  }
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  visual_tools_->publishTrajectoryLine(trajectory,
                                       move_group_->getCurrentState()->getJointModelGroup(active_planning_group));
  visual_tools_->trigger();
  publish_instruction_text(prompt);
  auto result = execute_movement(trajectory);
  if (!result.has_value())
    return tl::make_unexpected(result.error());
  return {};
}

tl::expected<void, std::string> DRTBehavior::execute_movement(moveit_msgs::msg::RobotTrajectory trajectory)
{
  if (cancel_behaviors)
  {
    cancel_behaviors = false;
    return tl::make_unexpected("Not executing the move because the STOP flag was set.");
  }
  move_group_->execute(trajectory);
  return {};
}

tl::expected<void, std::string> DRTBehavior::update_collision_matrix(std::string scene_object, std::string robot_link,
                                                                     CollisionType allow_collisions)
{
  // convert enum to bool to use
  bool allowed = (allow_collisions == CollisionType::Allow);

  // Get planning scene
  RCLCPP_INFO(logger, "Requesting planning scene.");
  auto get_planning_scene_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto response =
      this->request_response<moveit_msgs::srv::GetPlanningScene>(get_planning_scene_client_, get_planning_scene_req);

  // Modify ACM
  auto acm = collision_detection::AllowedCollisionMatrix(response->scene.allowed_collision_matrix);

  // if the scene object doesn't exist yet, add it
  if (!acm.hasEntry(scene_object))
  {
    RCLCPP_INFO(logger, "The entry %s does not exist. Adding it.", scene_object.c_str());
    acm.setEntry(scene_object, false);
  }
  if (robot_link == "")
  {
    acm.setEntry(scene_object, allowed);
  }
  else
  {
    // if the robot link doesn't exist, add it. This can be because it has recently been attached
    if (!acm.hasEntry(robot_link))
    {
      RCLCPP_INFO(logger, "The entry %s does not exist. Adding it.", robot_link.c_str());
      acm.setEntry(robot_link, false);
    }
    acm.setEntry(scene_object, robot_link, allowed);
  }
  std::vector<std::string> names;
  acm.getAllEntryNames(names);
  for (auto& name : names)
  {
    RCLCPP_INFO(logger, "entry: %s", name.c_str());
  }

  auto apply_planning_scene_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
  acm.getMessage(acm_msg);
  apply_planning_scene_req->scene.allowed_collision_matrix = acm_msg;
  apply_planning_scene_req->scene.is_diff = true;
  // Apply planning scene
  RCLCPP_INFO(logger, "Applying planning scene.");
  auto apply_response = this->request_response<moveit_msgs::srv::ApplyPlanningScene>(apply_planning_scene_client_,
                                                                                     apply_planning_scene_req);
  if (!apply_response->success)
  {
    if (robot_link == "")
    {
      return tl::make_unexpected("Failed to disable collisions with " + scene_object + ".");
    }
    else
    {
      return tl::make_unexpected("Failed to disable collisions between " + scene_object + " and " + robot_link + ".");
    }
  }
  return {};
}

tl::expected<void, std::string> DRTBehavior::create_collision_object(std::string object_id, std::string reference_frame,
                                                                     std::string mesh_filepath,
                                                                     geometry_msgs::msg::Pose pose,
                                                                     const Eigen::Vector3d& scale)
{
  bool valid_transform = tf_buffer_->canTransform("base_link", reference_frame, tf2::TimePointZero);
  if (!valid_transform)
    return tl::make_unexpected("was not able to find the transform for: " + reference_frame);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = reference_frame;
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_filepath, scale);

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  moveit_msgs::msg::ObjectColor color;
  color.id = object_id;
  color.color.r = 0.5;
  color.color.g = 0.5;
  color.color.b = 0.5;
  color.color.a = 1.0;
  planning_scene_interface_->applyCollisionObjects({ collision_object }, { color });
  rclcpp::Rate sleep_hz(2.0);
  sleep_hz.sleep();
  auto object_names = planning_scene_interface_->getKnownObjectNames();
  auto result = std::count(object_names.begin(), object_names.end(), object_id) > 0;
  if (!result)
    RCLCPP_WARN(logger, "Unsure if the collision object %s was applied", object_id.c_str());
  return {};
}

tl::expected<void, std::string> DRTBehavior::create_collision_object(std::string collision_object_name)
{
  auto node = config_yaml["meshes"][collision_object_name];
  geometry_msgs::msg::Pose pose;
  auto tf = get_tf_from_yaml(node);
  if (!tf.has_value())
    return tl::make_unexpected(tf.error());

  pose.orientation = tf.value().rotation;
  pose.position.x = tf.value().translation.x;
  pose.position.y = tf.value().translation.y;
  pose.position.z = tf.value().translation.z;

  std::string object_id = node["id"].as<std::string>();
  std::string mesh_filepath = node["mesh_filepath"].as<std::string>();
  std::string reference = node["reference"].as<std::string>();
  Eigen::Vector3d scale(node["scale"]["x"].as<double>(), node["scale"]["y"].as<double>(),
                        node["scale"]["z"].as<double>());
  return create_collision_object(object_id, reference, mesh_filepath, pose, scale);
}