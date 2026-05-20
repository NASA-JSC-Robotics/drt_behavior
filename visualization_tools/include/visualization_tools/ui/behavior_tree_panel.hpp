#pragma once

// STL
#include <chrono>
#include <memory>
#include <string>
// ROS2
#include <rclcpp/rclcpp.hpp>
// RVIZ2
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QGroupBox>
#include <QScrollArea>
#include <QTabWidget>
#include <QtWidgets>

#include <ui_rviz2_panel.h>
#include <visualization_tools/ui/behavior_tree_widget.hpp>

namespace visualization_tools
{
class BehaviorTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget* parent = nullptr);
  void onInitialize() override;

protected:
  // ROS Stuff
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> abstract_node_ptr_;

  // Widgets
  std::shared_ptr<BehaviorTreeWidget> behavior_tree_widget_;

private:
  std::unique_ptr<Ui::rviz_panel> ui_;
  // private Q_SLOTS:
};
};  // namespace visualization_tools
