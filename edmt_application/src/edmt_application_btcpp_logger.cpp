#include "edmt_application/edmt_application_btcpp_logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
static const auto kLogger = rclcpp::get_logger("edmt_btcpp_logger");
}

EdmtApplicationBtcppLogger::EdmtApplicationBtcppLogger(const BT::Tree& tree) : BT::StatusChangeLogger(tree.rootNode())
{
  // Build a lookup table for subtree ID -> subtree root
  std::map<std::string, const BT::Tree::Subtree*> subtree_map;
  for (const auto& st : tree.subtrees)
  {
    subtree_map[st->tree_ID] = st.get();
  }

  int counter = 0;
  std::cout << "=== Expanded Full Tree ===" << std::endl;
  printTree(tree.rootNode(), subtree_map, counter);
}
EdmtApplicationBtcppLogger::~EdmtApplicationBtcppLogger()
{
}

void EdmtApplicationBtcppLogger::printTree(const BT::TreeNode* node,
                                           const std::map<std::string, const BT::Tree::Subtree*>& subtree_map,
                                           int& counter, int indent)
{
  std::cout << std::string(indent, ' ') << "Node: " << node->name()  // instance name from XML (or type name)
            << " | Type: " << node->registrationName()               // node type
            << " | UID=" << node->UID()                              // BehaviorTree.CPP UID
            << " | CustomID=" << counter++                           // contiguous ID we assign
            << std::endl;

  // If this node is a SubTree, expand it
  if (auto subtree_node = dynamic_cast<const BT::SubTreeNode*>(node))
  {
    std::string target = subtree_node->subtreeID();
    auto it = subtree_map.find(target);
    if (it != subtree_map.end())
    {
      std::cout << std::string(indent + 2, ' ') << "-- Expanding SubTree: " << target << " --" << std::endl;
      printTree(it->second->root_node.get(), subtree_map, counter, indent + 2);
    }
  }
  else if (auto control = dynamic_cast<const BT::ControlNode*>(node))
  {
    for (const auto& child : control->children())
    {
      printTree(child, subtree_map, counter, indent + 2);
    }
  }
}

void EdmtApplicationBtcppLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                                          BT::NodeStatus status)
{
  using namespace std::chrono;

  constexpr const char* whitespaces = "                         ";
  constexpr const size_t ws_count = 25;

  double since_epoch = duration<double>(timestamp).count();
  printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
         &whitespaces[std::min(ws_count, node.name().size())], toStr(prev_status, true).c_str(),
         BT::toStr(status, true).c_str());
  std::cout << std::endl;

  std::string logmsg = node.fullPath() + "(" + std::to_string(node.UID()) + "): " + toStr(prev_status, true) + " -> " +
                       BT::toStr(status, true);

  RCLCPP_INFO(kLogger, logmsg.c_str());
}

void EdmtApplicationBtcppLogger::flush()
{
  std::cout << std::flush;
}
