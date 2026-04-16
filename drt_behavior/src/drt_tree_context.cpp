#include "drt_behavior/drt_tree_context.hpp"

namespace drt_behavior
{

std::shared_ptr<DRTTreeContext>
DRTTreeContext::create(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor, const std::string& node_name)
{
  auto context = std::make_shared<DRTTreeContext>();

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  // Create a new node... but this might be redundant and stupid. I don't actually know.
  // Why even bother with the upstream?
  context->node = std::make_shared<rclcpp::Node>(node_name, options);

  context->tf_buffer = std::make_shared<tf2_ros::Buffer>(context->node->get_clock());
  context->tf_listener = std::make_shared<tf2_ros::TransformListener>(*context->tf_buffer, context->node);
  // And it just spins...
  executor->add_node(context->node);
  return context;
}

void DRTTreeContext::teardown(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
{
  tf_listener.reset();
  tf_buffer.reset();

  if (node)
  {
    executor->remove_node(node);
    node.reset();
  }
}

}  // namespace drt_behavior
