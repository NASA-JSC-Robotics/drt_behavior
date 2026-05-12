#include "moveit_behavior/plan_to_pose.hpp"
namespace moveit_behavior
{
bool PlanToPose::setRequest(Request::SharedPtr& request)
{
  std::string group_name;
  if (!getInput("group_name", group_name))
  {
    throw BT::RuntimeError("Could not access required blackboard input [group_name]");
  }
  std::string end_effector_name;
  if (!getInput("end_effector_name", end_effector_name))
  {
    throw BT::RuntimeError("Could not access required blackboard input [end_effector_name]");
  }

  geometry_msgs::msg::PoseStamped goal_pose_stamped;
  if (!getInput("goal_pose", goal_pose_stamped))
  {
    throw BT::RuntimeError("Could not access required blackboard input [goal_pose]");
  }

  double position_tolerance;
  getInput("position_tolerance", position_tolerance);

  moveit_msgs::msg::WorkspaceParameters wp;
  wp.header.frame_id = "world";
  wp.min_corner.x = -10.0;
  wp.min_corner.y = -10.0;
  wp.min_corner.z = -10.0;
  wp.max_corner.x = 10.0;
  wp.max_corner.y = 10.0;
  wp.max_corner.z = 10.0;

  moveit_msgs::msg::Constraints constraints;

  moveit_msgs::msg::PositionConstraint pos_con;
  {
    shape_msgs::msg::SolidPrimitive pri;
    pri.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    pri.dimensions = { position_tolerance };

    geometry_msgs::msg::Pose pri_pose;
    pri_pose.position = goal_pose_stamped.pose.position;
    pri_pose.orientation.w = 1.0;

    pos_con.constraint_region.primitives.push_back(pri);
    pos_con.constraint_region.primitive_poses.push_back(pri_pose);

    pos_con.weight = 1.0;

    pos_con.link_name = end_effector_name;
    pos_con.header.frame_id = goal_pose_stamped.header.frame_id;
  }
  constraints.position_constraints.push_back(pos_con);

  std::vector<double> orientation_tolerance;
  if (getInput("orientation_tolerance", orientation_tolerance))
  {
    moveit_msgs::msg::OrientationConstraint ori_con;
    ori_con.header.frame_id = goal_pose_stamped.header.frame_id;
    ori_con.orientation = goal_pose_stamped.pose.orientation;

    ori_con.link_name = end_effector_name;
    ori_con.absolute_x_axis_tolerance = orientation_tolerance[0];
    ori_con.absolute_y_axis_tolerance = orientation_tolerance[1];
    ori_con.absolute_z_axis_tolerance = orientation_tolerance[2];

    ori_con.parameterization = moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;

    constraints.orientation_constraints.push_back(ori_con);
  }

  request->motion_plan_request.num_planning_attempts = 10;
  request->motion_plan_request.workspace_parameters = wp;
  request->motion_plan_request.group_name = group_name;

  request->motion_plan_request.goal_constraints.push_back(constraints);
  request->motion_plan_request.allowed_planning_time = 10.0;

  getInput("velocity_scaling", request->motion_plan_request.max_velocity_scaling_factor);
  getInput("acceleration_scaling", request->motion_plan_request.max_acceleration_scaling_factor);

  getInput("planner", request->motion_plan_request.planner_id);
  getInput("pipeline", request->motion_plan_request.pipeline_id);
  return true;
}

BT::NodeStatus PlanToPose::onResponseReceived(const Response::SharedPtr& response)
{
  moveit_msgs::msg::MotionPlanResponse mpr = response.get()->motion_plan_response;

  if (mpr.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    setOutput("trajectory", mpr.trajectory);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus PlanToPose::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
}  // namespace moveit_behavior
