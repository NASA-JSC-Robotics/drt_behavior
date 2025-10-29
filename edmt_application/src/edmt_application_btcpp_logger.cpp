#include "edmt_application/edmt_application_btcpp_logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("edmt_btcpp_logger");
}

EdmtApplicationBtcppLogger::EdmtApplicationBtcppLogger(const BT::Tree& tree, std::shared_ptr<rclcpp::Node> node)
  : BT::StatusChangeLogger(tree.rootNode()), node_(node)
{
  // Create a publisher so that we can
  bt_status_publisher_ = node_->create_publisher<edmt_application_msgs::msg::VectorOfStrings>("/bt_status", 10);

  generateTree(tree.rootNode());
}
EdmtApplicationBtcppLogger::~EdmtApplicationBtcppLogger()
{
}

void EdmtApplicationBtcppLogger::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                                          BT::NodeStatus prev_status, BT::NodeStatus status)
{
  // if we have failed, return early
  if (done_)
    return;

  // we want to stay shown as success, not show idle again, so leave it as is
  if (prev_status == BT::NodeStatus::SUCCESS && status == BT::NodeStatus::IDLE)
    return;

  log_statuses_[node.UID()].status = status;

  edmt_application_msgs::msg::VectorOfStrings logmsg_vec;
  logmsg_vec.data = {};
  // should be print tree function
  for (uint16_t id : log_order_)
  {
    std::string indent_string(log_statuses_[id].indent, '  ');
    std::string logmsg = indent_string + log_statuses_[id].name + ": " + toStr(log_statuses_[id].status, true);
    logmsg_vec.data.push_back(logmsg);
    // RCLCPP_INFO(kLogger, logmsg.c_str());
  }
  bt_status_publisher_->publish(logmsg_vec);
  // we don't want to print anymore if we have failed
  done_ = (status == BT::NodeStatus::FAILURE);
}

void EdmtApplicationBtcppLogger::generateTree(const BT::TreeNode* node, int indent)
{
  // on the first run, clear these data
  if (indent == 0)
  {
    log_statuses_.clear();
    log_order_.clear();
  }

  auto node_status = NodeLogStatus(node->name(), BT::NodeStatus::IDLE, indent);
  log_statuses_[node->UID()] = node_status;
  log_order_.push_back(node->UID());

  // iterate recursively through children to add them to the things
  if (auto control = dynamic_cast<const BT::ControlNode*>(node))
  {
    for (const auto& child : control->children())
    {
      generateTree(static_cast<const BT::TreeNode*>(child), indent + 1);
    }
  }
  else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
  {
    generateTree(decorator->child(), indent + 1);
  }
}

void EdmtApplicationBtcppLogger::flush()
{
  std::cout << std::flush;
}
