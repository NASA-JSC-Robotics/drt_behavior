#include "edmt_application/behaviors/plan_relative_move.hpp"
#include "edmt_application/edmt_application.hpp"

PlanRelativeMove::PlanRelativeMove(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList PlanRelativeMove::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

           // input params
           BT::InputPort<std::string>("relative_move_name"),
           BT::InputPort<std::string>("move_type", "cartesian", "default is cartesian"),
           BT::InputPort<float>("speed_scale", float{ 1.0 }, "default value is 1.0"),
           // output params
           BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory")
  };
}

// You must override the virtual function tick()
BT::NodeStatus PlanRelativeMove::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::string relative_move_name, move_type;
  float speed_scale;
  if (!getInput("relative_move_name", relative_move_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [relative_move_name]");
  }
  if (!getInput("move_type", move_type))
  {
    throw BT::RuntimeError("Could not access blackboard input [move_type]");
  }
  if (!getInput("speed_scale", speed_scale))
  {
    throw BT::RuntimeError("Could not access blackboard input [speed_scale]");
  }

  EdmtApplication::PlanType move_type_enum =
      move_type == "cartesian" ? EdmtApplication::PlanType::Cartesian : EdmtApplication::PlanType::Joint;
  auto trajectory = edmt_application_node_->plan_relative_move(relative_move_name, move_type_enum, speed_scale);

  if (trajectory.has_value())
  {
    setOutput("trajectory", trajectory.value());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    throw BT::RuntimeError(trajectory.error());
    return BT::NodeStatus::FAILURE;
  }
}