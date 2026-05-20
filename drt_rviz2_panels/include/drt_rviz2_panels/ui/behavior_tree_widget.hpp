#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// BT 
#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "btcpp_ros2_interfaces/srv/get_trees.hpp"
// RVIZ2
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
// Qt
#include <QtWidgets>
#include <QString>
// STL
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_behavior_tree_widget.h>
#include <drt_rviz2_panels/ui/base_rviz_widget.hpp>

namespace drt_rviz2_panels
{
    using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    class BehaviorTreeWidget : public BaseRVizWidget
    {
        Q_OBJECT

    public:
        explicit BehaviorTreeWidget(QWidget *parent,rclcpp::Node::SharedPtr node_ptr);

        ~BehaviorTreeWidget() = default;

        // #region OVERRIDES

        std::string getName() const override;

        void load(const rviz_common::Config &config) override;

        void save(rviz_common::Config config) const override;

        // #endregion OVERRIDES

        // #region PRIVATE

    private:
        const std::string green = "\033[92m";
        const std::string red = "\033[91m";
        const std::string blue = "\033[96m";
        const std::string yellow = "\033[93m";
        const std::string end_color = "\033[0m";

        std::string active_behavior = "";
        // #tag GUI
        std::unique_ptr<Ui::behavior_tree_widget> ui_;

        std::vector<std::string> available_behaviors;

        // #tag Class_Members
        // Widget Initialized
        bool widget_initialized_ = false;

        std::vector<std::string> log;
        std::vector<std::string> bt_text;

        rclcpp_action::Client<btcpp_ros2_interfaces::action::ExecuteTree>::SharedPtr execute_tree_client_;

        // rclcpp::Client<drt_behavior_msgs::srv::GetBehaviorTrees>::SharedPtr get_behavior_trees_client_;
        rclcpp::Client<btcpp_ros2_interfaces::srv::GetTrees>::SharedPtr get_behavior_trees_client_;
        
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr logger_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_status_subscriber_;

        // methods
        void initialize_ui();
        // callbacks

        void result_callback(const GoalHandleExecuteTree::WrappedResult &result);
        void goal_response_callback(const GoalHandleExecuteTree::SharedPtr &goal_handle);

        void get_behavior_trees_cb(
            const rclcpp::Client<btcpp_ros2_interfaces::srv::GetTrees>::SharedFuture future);

        void logger_callback(const std_msgs::msg::String::SharedPtr msg);
        void bt_status_callback(const std_msgs::msg::String::SharedPtr msg);

        std::string replace_string(std::string string_to_replace, std::string old_text, std::string new_text);

        // Update panel data
        void update_bt_text();
        void update_log_text();
        std::string process_text(std::string text);

    private Q_SLOTS:
        void on_StopButton_pressed();
        void on_InitializeButton_pressed();
        void on_RunButton_pressed();
    };

} // drt_rviz2_panels
