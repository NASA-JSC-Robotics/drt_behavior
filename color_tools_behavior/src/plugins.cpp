#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "color_tools_behavior/color_blob_find.hpp"
#include "color_tools_behavior/detect_color_blobs.hpp"
#include "color_tools_behavior/get_synced_image_pointcloud_depth.hpp"

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<color_tools_behavior::ColorBlobFind>("ColorBlobFind", params);
  factory.registerNodeType<color_tools_behavior::GetSyncedImagePointCloudDepth>("GetSyncedImagePointCloudDepth");
  factory.registerNodeType<color_tools_behavior::DetectColorBlobs>("DetectColorBlobs");
}
