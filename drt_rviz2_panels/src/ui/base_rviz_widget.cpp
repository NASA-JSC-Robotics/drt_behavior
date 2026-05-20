#include <drt_rviz2_panels/ui/base_rviz_widget.hpp>

namespace drt_rviz2_panels
{

BaseRVizWidget::BaseRVizWidget(
  QWidget * parent,
  rclcpp::Node::SharedPtr node_ptr
)
: QWidget(parent),
  node_(node_ptr)
{

  rclcpp::Node::SharedPtr temp_node = std::make_shared<rclcpp::Node>("temp");
  temp_node.reset();

}

// #region PROTECTED

void BaseRVizWidget::waitForClient(rclcpp::ClientBase::SharedPtr client)
{
  // Get the name of the client
  std::string client_name = client->get_service_name();
  // Wait for the client to be ready
  while (!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Service %s not available, waiting again...",
      client_name.c_str());
  }
  RCLCPP_DEBUG(
    node_->get_logger(), "Service %s is now available", client_name.c_str());
}

// #endregion PROTECTED

}  // namespace rviz2_panel
