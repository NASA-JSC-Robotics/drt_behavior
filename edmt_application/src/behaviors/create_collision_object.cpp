#include "edmt_application/behaviors/create_collision_object.hpp"
#include "edmt_application/edmt_application.hpp"

CreateCollisionObject::CreateCollisionObject(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

// It is mandatory to define this STATIC method.
BT::PortsList CreateCollisionObject::providedPorts()
{
    return {
        // global param
        BT::InputPort<std::shared_ptr<EdmtApplication>>("edmt_application_node", "{@edmt_application_node}"),

        // input params
        BT::InputPort<std::string>("collision_object_name"),
    };
}

// You must override the virtual function tick()
BT::NodeStatus CreateCollisionObject::tick()
{
    std::shared_ptr<EdmtApplication> edmt_application_node_;
    if (!getInput("edmt_application_node", edmt_application_node_)) {
        throw BT::RuntimeError("Could not access global blackboard input [edmt_application_node]");
    }

    std::string collision_object_name;
    if (!getInput("collision_object_name", collision_object_name)) {
        throw BT::RuntimeError("Could not access blackboard input [collision_object_name]");
    }

    auto result = edmt_application_node_->create_collision_object(collision_object_name);

    if (result.has_value()){
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::FAILURE;
    }

}