#include "edmt_application/behaviors/update_collision_matrix.hpp"
#include "edmt_application/edmt_application.hpp"

UpdateCollisionMatrix::UpdateCollisionMatrix(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList UpdateCollisionMatrix::providedPorts()
{
  return { // global param
           BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

           // input params
           BT::InputPort<std::string>("scene_object"), BT::InputPort<std::string>("robot_link"),
           BT::InputPort<std::string>("allow_collisions")
  };
}

// You must override the virtual function tick()
BT::NodeStatus UpdateCollisionMatrix::tick()
{
  std::shared_ptr<EdmtApplication> edmt_application_node_;
  if (!getInput("edmt_application_node", edmt_application_node_))
  {
    throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
  }

  std::string scene_object, robot_link, allow_collisions;
  if (!getInput("scene_object", scene_object))
  {
    throw BT::RuntimeError("Could not access blackboard input [scene_object]");
  }
  if (!getInput("robot_link", robot_link))
  {
    throw BT::RuntimeError("Could not access blackboard input [robot_link]");
  }
  if (!getInput("allow_collisions", allow_collisions))
  {
    throw BT::RuntimeError("Could not access blackboard input [allow_collisions]");
  }
  if (!(allow_collisions == "allow" || allow_collisions == "Allow" || allow_collisions == "disallow" ||
        allow_collisions == "Disallow"))
  {
    throw BT::RuntimeError("allow_collisions must be '[Aa]llow' or '[Dd]isallow'");
  }

  EdmtApplication::CollisionType allow_collisions_enum = (allow_collisions == "allow" || allow_collisions == "Allow") ?
                                                             EdmtApplication::CollisionType::Allow :
                                                             EdmtApplication::CollisionType::Disallow;

  auto result = edmt_application_node_->update_collision_matrix(scene_object, robot_link, allow_collisions_enum);

  if (result.has_value())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}