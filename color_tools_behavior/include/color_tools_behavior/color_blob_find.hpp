#pragma once

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "color_tools_msgs/srv/blob_centroid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace color_tools_behavior
{
using ClrBlbFind = color_tools_msgs::srv::BlobCentroid;

class ColorBlobFind : public BT::RosServiceNode<ClrBlbFind>
{
public:
  explicit ColorBlobFind(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
    : BT::RosServiceNode<ClrBlbFind>(name, config, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<int>("desired_blob", 0, "Desired Blob index. Default is 0"),
          BT::InputPort<double>("min_blob_size", 10.0, "Minimum size of width or height of object in pixels"),
          BT::InputPort<std::string>("color", "red", "color of the mask to use"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>("centroid_pose", "Centroid of the found blob") });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};

};  // namespace color_tools_behavior
