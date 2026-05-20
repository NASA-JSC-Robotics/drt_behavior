#include "drt_rviz2_panels/ui/behavior_tree_widget.hpp"

namespace drt_rviz2_panels
{
    using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
    using GoalHandleExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    BehaviorTreeWidget::BehaviorTreeWidget(
        QWidget *parent,
        rclcpp::Node::SharedPtr node_ptr)
        : BaseRVizWidget{parent, node_ptr},
          ui_(std::make_unique<Ui::behavior_tree_widget>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        initialize_ui();

        // #tag ROS_Service_Clients
        get_behavior_trees_client_ = node_->create_client<btcpp_ros2_interfaces::srv::GetTrees>("/get_loaded_trees");

        stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/drt_behavior_stop");

        bt_status_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "/bt_action_server/bt_status",
            10,
            std::bind(&BehaviorTreeWidget::bt_status_callback,
                      this,
                      std::placeholders::_1));

        execute_tree_client_ = rclcpp_action::create_client<ExecuteTree>(node_, "/bt_execution");

        // Set the initialized status
        // Right when we finish constructing the widget let actually send the request to get behavior trees
        on_InitializeButton_pressed();
    }

    void BehaviorTreeWidget::initialize_ui()
    {
        ui_->AutoScrollLog->setChecked(true);
        ui_->AutoScrollBehaviorTree->setChecked(false);
    }

    // #region OVERRIDES

    void BehaviorTreeWidget::on_InitializeButton_pressed()
    {
        widget_initialized_ = false;
        // Wait for the service to be available
        if (!get_behavior_trees_client_->wait_for_service(std::chrono::milliseconds(500)))
        {
            RCLCPP_ERROR(node_->get_logger(), "list_behavior_trees service was not available! Not initializing");
            return;
        }

        auto request = std::make_shared<btcpp_ros2_interfaces::srv::GetTrees::Request>();
        auto future = get_behavior_trees_client_->async_send_request(request, std::bind(&BehaviorTreeWidget::get_behavior_trees_cb, this, std::placeholders::_1));
        return;
    }

    void BehaviorTreeWidget::on_StopButton_pressed()
    {
        std::cout << "Canceling current goal..." << std::endl;
        execute_tree_client_->async_cancel_all_goals();
        return;
    }

    void BehaviorTreeWidget::on_RunButton_pressed()
    {
        if (ui_->SelectBehaviorComboBox->count() == 0)
        {
            RCLCPP_WARN(node_->get_logger(), "Behavior combo box is empty.");
            return;
        }

        int selected_index = ui_->SelectBehaviorComboBox->currentIndex();
        active_behavior = available_behaviors[selected_index];

        RCLCPP_INFO(node_->get_logger(), "Sending goal: %s", active_behavior.c_str());

        auto goal_msg = ExecuteTree::Goal();
        goal_msg.target_tree = active_behavior;

        auto send_goal_options = rclcpp_action::Client<ExecuteTree>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&BehaviorTreeWidget::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&BehaviorTreeWidget::result_callback, this, std::placeholders::_1);
        execute_tree_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void BehaviorTreeWidget::goal_response_callback(const GoalHandleExecuteTree::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            return;
        }
        std::string current_tree_text = "Current Tree: <b>" + active_behavior + "</b>";
        ui_->CurrentTreeText->setText(QString::fromStdString(current_tree_text));
    }

    void BehaviorTreeWidget::result_callback(const GoalHandleExecuteTree::WrappedResult &result)
    {
        std::string last_behavior_run_text = "Last Behavior Run: <b>" + active_behavior + "</b> (";
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            last_behavior_run_text += "<font color=\"Green\">SUCCEEDED</font>)";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            last_behavior_run_text += "<font color=\"Red\">ABORTED</font>)";
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            last_behavior_run_text += "<font color=\"Red\">CANCELLED</font>)";
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            break;
        }

        auto log_msg = std::make_shared<std_msgs::msg::String>();
        log_msg->data = "Tree result: " + result.result->return_message;
        logger_callback(log_msg);

        ui_->LastBehaviorRunText->setText(QString::fromStdString(last_behavior_run_text));
    }

    void BehaviorTreeWidget::get_behavior_trees_cb(
        const rclcpp::Client<btcpp_ros2_interfaces::srv::GetTrees>::SharedFuture future)
    {
        // Update the widget status
        available_behaviors = future.get()->tree_ids;

        ui_->SelectBehaviorComboBox->clear();
        for (const auto &item : available_behaviors)
        {
            auto item_qstr = QString::fromStdString(item);
            ui_->SelectBehaviorComboBox->addItem(item_qstr, item_qstr);
        }
        widget_initialized_ = true;
        return;
    }

    // publisher callbacks
    void BehaviorTreeWidget::bt_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<std::string> result;
        std::stringstream ss(msg->data);
        std::string item;
        while (std::getline(ss, item, ';')) {
            result.push_back(item);
        }
        bt_text = result;
        update_bt_text();
    }

    void BehaviorTreeWidget::logger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto time = node_->get_clock()->now();
        std::string log_string = "[" + std::to_string(time.seconds()) + "] " + msg->data;
        log.push_back(log_string);

        update_log_text();
    }

    std::string BehaviorTreeWidget::process_text(std::string text)
    {
        // terminal green
        text = replace_string(text, green, "<font color=\"Green\">");
        // btcpp green
        text = replace_string(text, "\x1b[32m", "<font color=\"Green\">");
        // terminal red
        text = replace_string(text, red, "<font color=\"Red\">");
        // btcpp red
        text = replace_string(text, "\x1b[31m", "<font color=\"Red\">");
        // terminal blue
        text = replace_string(text, blue, "<font color=\"Blue\">");
        // btcpp blue
        text = replace_string(text, "\x1b[34m", "<font color=\"Blue\">");
        // terminal yellow
        text = replace_string(text, yellow, "<font color=\"Orange\">");
        // btcpp yelow
        text = replace_string(text, "\x1b[33m", "<font color=\"Orange\">");
        // btcpp cyan
        text = replace_string(text, "\x1b[36m", "<font color=\"Cyan\">");
        // terminal end color
        text = replace_string(text, end_color, "</font>");
        // btcpp end color
        text = replace_string(text, "\x1b[0m", "</font>");
        text = replace_string(text, "\t", "  ");
        text = replace_string(text, "│   ", "|&nbsp;&nbsp;&nbsp;");
        text = replace_string(text, "    ", "&nbsp;&nbsp;&nbsp;&nbsp;");
        text = replace_string(text, "\n", "<br/>");
        text = replace_string(text, "\r", "<br/>");
        return text;
    }

    void BehaviorTreeWidget::update_bt_text()
    {
        bool first_time = true;
        std::string bt_print_text = "";

        for (const auto &bt_data : bt_text)
        {
            if (first_time)
            {
                first_time = false;
            }
            else
            {
                bt_print_text += "<br/>";
            }
            bt_print_text += process_text(bt_data);
        }

        auto formatted_bt_print_text = "<pre>" + bt_print_text + "</pre>";

        QTextCursor cursor = ui_->BehaviorTreeText->textCursor();
        // int lineNumber = cursor.blockNumber();
        auto scroll_value = ui_->BehaviorTreeText->verticalScrollBar()->value();

        ui_->BehaviorTreeText->setHtml(QString::fromStdString(formatted_bt_print_text));

        if (ui_->AutoScrollBehaviorTree->isChecked())
        {
            // cursor.movePosition(QTextCursor::End);
            // ui_->BehaviorTreeText->setTextCursor(cursor);
            // Get the full text of the document
            QTextDocument *doc = ui_->BehaviorTreeText->document();

            QString fullText = doc->toPlainText();
            std::vector<QString> searchTerms = {"SUCCESS", "FAILURE", "RUNNING", "SKIPPED"}; // substring to find

            int overall_last_index = -1;
            // Find last occurrence of anything that isn't IDLE that we can use as a autoscroll position
            for (auto &term : searchTerms)
            {
                int lastIndex = fullText.lastIndexOf(term, -1, Qt::CaseInsensitive);
                if (lastIndex > overall_last_index)
                {
                    overall_last_index = lastIndex;
                }
            }

            if (overall_last_index != -1)
            {
                // Move cursor to that position
                QTextCursor cursor = ui_->BehaviorTreeText->textCursor();
                cursor.setPosition(overall_last_index);
                ui_->BehaviorTreeText->setTextCursor(cursor);

                // Scroll so it’s visible
                ui_->BehaviorTreeText->ensureCursorVisible();
            }
        }
        else
        {
            ui_->BehaviorTreeText->verticalScrollBar()->setValue(scroll_value);
        }
    }

    void BehaviorTreeWidget::update_log_text()
    {
        bool first_time = true;
        std::string log_text = "";

        for (const auto &log_data : log)
        {
            if (first_time)
            {
                first_time = false;
            }
            else
            {
                log_text += "<br/>";
            }
            log_text += process_text(log_data);
        }

        auto formatted_log_text = "<pre style='white-space: pre-wrap;'>" + log_text + "</pre>";

        ui_->LogText->setHtml(QString::fromStdString(formatted_log_text));

        if (ui_->AutoScrollLog->isChecked())
        {
            QTextCursor cursor = ui_->LogText->textCursor();
            cursor.movePosition(QTextCursor::End);
            ui_->LogText->setTextCursor(cursor);
        }
    }

    std::string BehaviorTreeWidget::replace_string(std::string string_to_replace, std::string old_text, std::string new_text)
    {
        size_t pos = 0;
        while ((pos = string_to_replace.find(old_text, pos)) != std::string::npos)
        {
            string_to_replace.replace(pos, old_text.length(), new_text);
            pos += new_text.length(); // Move past the replaced part
        }
        return string_to_replace;
    }

    std::string BehaviorTreeWidget::getName() const
    {
        return "behavior_tree_widget";
    }

    void BehaviorTreeWidget::load(const rviz_common::Config &/*config*/)
    {
        // TODO: Implement
    }

    void BehaviorTreeWidget::save(rviz_common::Config /*config*/) const
    {
        // TODO: Implement
    }

} // drt_rviz2_panels
