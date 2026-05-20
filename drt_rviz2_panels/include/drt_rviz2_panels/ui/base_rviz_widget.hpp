#pragma once

// STL
#include <string>
#include <chrono>
#include <memory>
// ROS
#include <rclcpp/rclcpp.hpp>
// RVIZ2
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QtWidgets>

namespace drt_rviz2_panels
{

class BaseRVizWidget : public QWidget
{
  // Q_OBJECT

public:
  explicit BaseRVizWidget(
    QWidget * parent,
    rclcpp::Node::SharedPtr node_ptr
  );

  ~BaseRVizWidget() = default;

  virtual std::string getName() const = 0;

  virtual void load(const rviz_common::Config & config) = 0;

  virtual void save(rviz_common::Config config) const = 0;

protected:
  // ROS
  rclcpp::Node::SharedPtr node_;

  void waitForClient(rclcpp::ClientBase::SharedPtr client);


};

};  // namespace rviz2_panel
