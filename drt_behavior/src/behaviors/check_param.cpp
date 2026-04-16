#include "drt_behavior/behaviors/check_param.hpp"

CheckParam::CheckParam(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList CheckParam::providedPorts()
{
  return { // input params
           BT::InputPort<std::string>("param_name")
  };
}

BT::NodeStatus CheckParam::onStart()
{
  if (!getInput("param_name", param_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [param_name]");
  }

  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  bool param_value = false;
  context->node->get_parameter_or(param_name, param_value, false);
  if (param_value)
  {
    context->node->set_parameter(rclcpp::Parameter(param_name, false));
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus CheckParam::onRunning()
{
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  bool param_value = false;
  context->node->get_parameter_or(param_name, param_value, false);
  if (param_value)
  {
    // context->node->undeclare_parameter(param_name);
    context->node->set_parameter(rclcpp::Parameter(param_name, false));
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void CheckParam::onHalted()
{ std::cout << "Interrupted [CheckParam] BT node" << std::endl; }
