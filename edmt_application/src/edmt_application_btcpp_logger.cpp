#include "edmt_application/edmt_application_btcpp_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("edmt_btcpp_logger");
}

EdmtApplicationBtcppLogger::EdmtApplicationBtcppLogger(const BT::Tree& tree) : BT::StatusChangeLogger(tree.rootNode())
{}
EdmtApplicationBtcppLogger::~EdmtApplicationBtcppLogger()
{}

void EdmtApplicationBtcppLogger::callback(BT::Duration timestamp, const BT::TreeNode& node,
                             BT::NodeStatus prev_status, BT::NodeStatus status)
{
  using namespace std::chrono;

  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;

  double since_epoch = duration<double>(timestamp).count();
  printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
         &whitespaces[std::min(ws_count, node.name().size())],
         toStr(prev_status, true).c_str(), BT::toStr(status, true).c_str());
  std::cout << std::endl;

  std::string logmsg = node.fullPath() + ": " + toStr(prev_status, true) + " -> " + BT::toStr(status, true);

  RCLCPP_INFO(kLogger, logmsg.c_str());
}

void EdmtApplicationBtcppLogger::flush()
{
  std::cout << std::flush;
}
