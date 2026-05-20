#include "drt_rviz2_panels/ui/behavior_tree_panel.hpp"

namespace drt_rviz2_panels
{
    BehaviorTreePanel::BehaviorTreePanel(QWidget *parent)
        : rviz_common::Panel{parent}, ui_(std::make_unique<Ui::rviz_panel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);
    }
    // Overridden from Panel
    void BehaviorTreePanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        abstract_node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // as per normal rclcpp code
        node_ = abstract_node_ptr_->get_raw_node();
        // Add the BehaviorTreeWidget to the GUI
        behavior_tree_widget_ = std::make_shared<BehaviorTreeWidget>(ui_->guiArea,
                                                                        node_);
        // ui_->guiArea = behavior_tree_widget_.get();
        ui_->guiArea->layout()->addWidget(behavior_tree_widget_.get());
        ui_->retranslateUi(this);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drt_rviz2_panels::BehaviorTreePanel, rviz_common::Panel)
