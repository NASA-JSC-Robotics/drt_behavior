#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_behavior/align_transform.hpp"
#include "geometry_behavior/apply_transform.hpp"
#include "geometry_behavior/async_tf_lookup.hpp"
#include "geometry_behavior/create_random_transform.hpp"
#include "geometry_behavior/publish_transform.hpp"
#include "geometry_behavior/transform_to_pose.hpp"

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<geometry_behavior::AlignTransform>("AlignTransform");
  factory.registerNodeType<geometry_behavior::ApplyTransform>("ApplyTransform");
  factory.registerNodeType<geometry_behavior::AsyncTfLookup>("AsyncTfLookup");
  factory.registerNodeType<geometry_behavior::TransformToPose>("TransformToPose");
  factory.registerNodeType<geometry_behavior::CreateRandomTransform>("CreateRandomTransform");
  factory.registerNodeType<geometry_behavior::PublishTransform>("PublishTransform");
}
