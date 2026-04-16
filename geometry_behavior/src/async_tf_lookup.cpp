#include "geometry_behavior/async_tf_lookup.hpp"

namespace geometry_behavior
{

AsyncTfLookup::AsyncTfLookup(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList AsyncTfLookup::providedPorts()
{
  return { // input params
           BT::InputPort<std::string>("base_frame"), BT::InputPort<std::string>("target_frame"),
           // output params
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("tf")
  };
}
BT::NodeStatus AsyncTfLookup::onStart()
{
  if (!getInput("base_frame", base_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [base_frame]");
  }
  if (!getInput("target_frame", target_frame))
  {
    throw BT::RuntimeError("Could not access blackboard input [target_frame]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  try
  {
    auto result = context->tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
    setOutput("tf", result);
    return BT::NodeStatus::SUCCESS;
  }
  catch (const tf2::TransformException& ex)
  {
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus AsyncTfLookup::onRunning()
{
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  try
  {
    auto result = context->tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
    setOutput("tf", result);
    RCLCPP_DEBUG(context->node->get_logger(), "Found transform : %s -> %s", base_frame.c_str(), target_frame.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(context->node->get_logger(), "Looking for transform : %s -> %s", base_frame.c_str(),
                 target_frame.c_str());
    return BT::NodeStatus::RUNNING;
  }
}

void AsyncTfLookup::onHalted()
{ std::cout << "Interrupted [AsyncTfLookup] BT node" << std::endl; }

}  // namespace geometry_behavior
