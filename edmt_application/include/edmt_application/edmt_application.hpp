#pragma once

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <controller_manager/controller_manager.hpp>
#include <functional>
#include <geometry_msgs/msg/wrench.hpp>
#include <memory>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl_expected/expected.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

class EdmtApplication : public rclcpp::Node
{
public:
  /**
   * @brief
   *
   */
  enum CollisionType
  {
    Disallow,
    Allow
  };

  enum PlanType
  {
    Cartesian,
    Joint
  };

  enum BehaviorStatus
  {
    Pending,
    Active,
    Success,
    Failure
  };

  class BehaviorItem
  {
  public:
    std::string name;
    std::shared_ptr<BehaviorItem> parent;
    std::vector<std::shared_ptr<BehaviorItem>> children;
    BehaviorStatus status;

    /**
     * @brief Construct a new Behavior Item object, which stores the name, parent, and children
     *
     * @param name name of the behavior
     */
    BehaviorItem(std::string name) : name(name), parent(nullptr), status(BehaviorStatus::Active)
    {
    }
  };
  std::shared_ptr<BehaviorItem> tree_head;
  std::shared_ptr<BehaviorItem> current_node;

  EdmtApplication(std::string default_planning_group, rclcpp::NodeOptions node_options);

  /**
   * @brief Set the currently active move group of the context
   *
   * @param move_group_name the name of the move group that we will use
   */
  void set_move_group(std::string move_group_name);

  /**
   * @brief look up a frame and convert it to a transform stamped for cartesian move
   *
   * @param target_frame the target frame to convert to from base_frame
   * @param base_frame the base_frame to convert from (Defaults to base_link)
   * @return geometry_msgs::msg::TransformStamped
   */
  geometry_msgs::msg::TransformStamped tf_lookup(std::string target_frame, std::string base_frame = "world");

  /**
   * @brief look up a frame and convert it to a pose for cartesian move
   *
   * @param target_frame the target frame to convert to from base_frame
   * @param base_frame the base_frame to convert from (Defaults to base_link)
   * @return * geometry_msgs::msg::Pose
   */
  geometry_msgs::msg::Pose tf_lookup_converted(std::string target_frame, std::string base_frame = "world");

  /**
   * @brief special function to run once at the beginning to initialze the context
   *
   */
  virtual void initialize();

  bool load_configs();

  /**
   * @brief Create a collision object
   *
   * @param object_id the name of the object that you are creating a collion object for
   * @param reference_frame the reference frame that the pose should be attached to
   * @param mesh_filepath the filepath of the mesh that you are loading
   * @param pose the pose of the object with respect to the reference frame
   * @param scale the scale to apply to the mesh (defaults to normal scale)
   * @return tl::expected<void, std::string> success of the application
   */
  tl::expected<void, std::string>
  create_collision_object(std::string object_id, std::string reference_frame, std::string mesh_filepath,
                          geometry_msgs::msg::Pose pose, const Eigen::Vector3d& scale = Eigen::Vector3d(1.0, 1.0, 1.0));

  /**
   * @brief Create a collision object
   *
   * @param collision_object_name the name of the string that the collision object is being created for, as
   * referenced in the config file
   * @return tl::expected<void, std::string> success of the application
   */
  tl::expected<void, std::string> create_collision_object(std::string collision_object_name);

  tl::expected<void, std::string> update_collision_matrix(std::string scene_object, std::string robot_link,
                                                          CollisionType allow_collisions);

  tl::expected<geometry_msgs::msg::Transform, std::string> get_tf_from_yaml(YAML::Node node);

  tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
  plan_cartesian_waypoint_pose(geometry_msgs::msg::Pose waypoints, float speed_scale = 1.0);

  tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
  plan_joint_waypoint_pose(geometry_msgs::msg::Pose waypoints, float speed_scale = 1.0);

  tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
  plan_relative_move(std::string relative_move_name, PlanType move_type, float speed_scale = 1.0);

  /**
   * @brief generic synchronous call to a ros service
   *
   * @tparam ClientType type of client
   * @param client the client to call
   * @param request the request to send
   * @return Response
   */
  template <typename ClientType>
  std::shared_ptr<typename ClientType::Response> request_response(std::shared_ptr<rclcpp::Client<ClientType>> client,
                                                                  std::shared_ptr<typename ClientType::Request> request)
  {
    auto future = client->async_send_request(request);
    // Wait for the result.
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("edmt_application"), *this->get_clock(), 1000,
                           "Waiting for service response...");
    }
    return future.get();
  }

  /**
   * @brief plan to a joint state defined in the config file provided for the application
   *
   * @param joint_state_name the name of the joint state in the srdf
   * @param speed_scale the speed scale to move at
   * @return tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
   */
  tl::expected<moveit_msgs::msg::RobotTrajectory, std::string> plan_joint_states(std::string joint_state_name,
                                                                                 float speed_scale = 1.0);

  /**
   * @brief plan to a named state as described in the srdf predefined joint states
   *
   * @param move_group the move group to plan for
   * @param state_name the name of the joint state in the srdf
   * @param speed_scale the speed scale to move at
   * @return tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
   */
  tl::expected<moveit_msgs::msg::RobotTrajectory, std::string>
  plan_named_state(std::string move_group, std::string state_name, float speed_scale);

  tl::expected<void, std::string> prompt_and_execute(moveit_msgs::msg::RobotTrajectory trajectory, std::string prompt);

  tl::expected<void, std::string> execute_movement(moveit_msgs::msg::RobotTrajectory trajectory);

  void publish_instruction_text(std::string prompt);

  void publish_instruction_text_nb(std::string prompt);

  void printTree(const std::shared_ptr<BehaviorItem>& item, const std::string& prefix = "", bool isLast = true);

  void make_behavior_tree();

  std::pair<bool, std::string> call_behavior(std::string behavior_name);

  std::unordered_map<std::string, std::function<std::pair<bool, std::string>()>> function_registry;

  std::string green = "\033[92m";
  std::string red = "\033[91m";
  std::string blue = "\033[96m";
  std::string yellow = "\033[93m";
  std::string end_color = "\033[0m";

  std::string local_behavior_tree;
  std::string behavior_tree;
  std::mutex behavior_tree_mutex;
  std::pair<int, std::string> instruction;
  std::mutex instruction_mutex;
  int instruction_counter = 0;

  std::string active_planning_group;

  std::atomic<bool> cancel_behaviors = false;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

  std::vector<std::string> active_behaviors;

  YAML::Node config_yaml;

  bool sim = true;
};