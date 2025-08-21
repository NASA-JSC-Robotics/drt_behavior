#pragma once

#include <cstring>
#include "behaviortree_cpp/loggers/abstract_logger.h"

class EdmtApplicationBtcppLogger : public BT::StatusChangeLogger
{
public:
  EdmtApplicationBtcppLogger(const BT::Tree& tree);
  ~EdmtApplicationBtcppLogger() override;

  virtual void flush() override;

private:
  virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                        BT::NodeStatus status) override;
};