#pragma once

#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "drt_behavior/drt_tree_context.hpp"

#include <spdlog/spdlog.h>

#include <color_blob_centroid/color_blob_centroid.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

namespace color_tools_behavior
{
/**
 * @brief Given a synchronized set of images and camera info, identify color blobs
 * as specified by the user.
 */
class DetectColorBlobs : public BT::ThreadedAction
{
public:
  DetectColorBlobs(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  color_blob_centroid::BlobRequest blob_request;
};
}  // namespace color_tools_behavior
