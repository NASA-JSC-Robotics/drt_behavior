/* Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <chrono>

#include <color_tools_behavior/get_synced_image_pointcloud_depth.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace color_tools_behavior
{

namespace
{
tl::expected<rmw_qos_profile_t, std::string>
getPublisherQoSProfile(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic_name,
                       std::chrono::seconds timeout = std::chrono::seconds{ 5 })
{
  const auto start_time = node->now();
  while (rclcpp::ok())
  {
    const auto publishers_info = node->get_publishers_info_by_topic(topic_name);

    if (!publishers_info.empty())
    {
      return publishers_info.front().qos_profile().get_rmw_qos_profile();
    }

    if (node->now() - start_time > rclcpp::Duration(timeout))
    {
      return tl::make_unexpected("Timed out waiting for publisher on topic: " + topic_name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return tl::make_unexpected("ROS shutdown while waiting for publisher on topic: " + topic_name);
}
}  // namespace

GetSyncedImagePointCloudDepth::GetSyncedImagePointCloudDepth(const std::string& name,
                                                             const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList GetSyncedImagePointCloudDepth::providedPorts()
{
  return { BT::InputPort<std::string>(kTopicNamePointCloud, "/wrist_camera/points",
                                      "Point cloud topic the Behavior subscribes to."),
           BT::InputPort<std::string>(kTopicNameRgbImage, "/wrist_camera/color",
                                      "RGB image topic the Behavior subscribes to."),
           BT::InputPort<std::string>(kTopicNameRgbCameraInfo, "/wrist_camera/camera_info",
                                      "RGB camera info topic the Behavior subscribes to."),
           BT::InputPort<std::string>(kTopicNameDepthImage, "/wrist_camera/depth",
                                      "Depth image topic the Behavior subscribes to."),
           BT::OutputPort<PointCloud2>(kOutputNamePointCloud, "{point_cloud}",
                                       "Point cloud time-synchronized with all other outputs, as a "
                                       "sensor_msgs::msg::PointCloud2 message."),
           BT::OutputPort<Image>(kOutputNameRgbImage, "{rgb_image}",
                                 "RGB image time-synchronized with all other outputs, as a sensor_msgs::msg::Image "
                                 "message."),
           BT::OutputPort<CameraInfo>(kOutputNameRgbCameraInfo, "{rgb_camera_info}",
                                      "RGB camera information time-synchronized with all other outputs, as a "
                                      "sensor_msgs::msg::CameraInfo message."),
           BT::OutputPort<Image>(kOutputNameDepthImage, "{depth_image}",
                                 "Depth image time-synchronized with all other outputs, as a sensor_msgs::msg::Image "
                                 "message.") };
}

BT::NodeStatus GetSyncedImagePointCloudDepth::onStart()
{
  sync_done_ = false;

  std::string point_cloud_topic_name, rgb_image_topic_name, rgb_camera_info_topic_name, depth_image_topic_name;

  getInput<std::string>(kTopicNamePointCloud, point_cloud_topic_name);
  getInput<std::string>(kTopicNameRgbImage, rgb_image_topic_name);
  getInput<std::string>(kTopicNameRgbCameraInfo, rgb_camera_info_topic_name);
  getInput<std::string>(kTopicNameDepthImage, depth_image_topic_name);

  // Start subscribers and synchronizer.
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  node_ = context->node;
  auto node_raw_ptr = context->node.get();

  image_transport::TransportHints hints(context->node.get(), "raw");

  // Get publisher QoS profiles
  auto maybe_point_cloud_qos_profile = getPublisherQoSProfile(context->node, point_cloud_topic_name);
  auto maybe_rgb_image_qos_profile = getPublisherQoSProfile(context->node, rgb_image_topic_name);
  auto maybe_rgb_camera_info_qos_profile = getPublisherQoSProfile(context->node, rgb_camera_info_topic_name);
  auto maybe_depth_image_qos_profile = getPublisherQoSProfile(context->node, depth_image_topic_name);

  // // Check that QoS profiles were retrieved.
  // if (const auto error =
  //         moveit_studio::behaviors::maybe_error(maybe_point_cloud_qos_profile, maybe_rgb_image_qos_profile,
  //         maybe_rgb_camera_info_qos_profile,
  //                     maybe_depth_image_qos_profile))
  // {
  //   shared_resources_->logger->publishFailureMessage(
  //       name(), std::string("Failed to get required publisher QoS profile: ").append(error.value()));
  //   return BT::NodeStatus::FAILURE;
  // }

  sub_point_cloud_.subscribe(node_raw_ptr, point_cloud_topic_name, maybe_point_cloud_qos_profile.value());
  sub_rgb_image_.subscribe(node_raw_ptr, rgb_image_topic_name, hints.getTransport(),
                           maybe_rgb_image_qos_profile.value());
  sub_rgb_camera_info_.subscribe(node_raw_ptr, rgb_camera_info_topic_name, maybe_rgb_camera_info_qos_profile.value());
  sub_depth_image_.subscribe(node_raw_ptr, depth_image_topic_name, hints.getTransport(),
                             maybe_depth_image_qos_profile.value());

  synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(kMessageSyncQueueSize), sub_point_cloud_, sub_rgb_image_,
                                                 sub_rgb_camera_info_, sub_depth_image_);
  synchronizer_->registerCallback(std::bind(&GetSyncedImagePointCloudDepth::syncCallback, this, std::placeholders::_1,
                                            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  start_time_ = context->node->now();

  return BT::NodeStatus::RUNNING;
}

void GetSyncedImagePointCloudDepth::unsubscribeTopics()
{
  sub_point_cloud_.unsubscribe();
  sub_rgb_image_.unsubscribe();
  sub_rgb_camera_info_.unsubscribe();
  sub_depth_image_.unsubscribe();
}

void GetSyncedImagePointCloudDepth::syncCallback(const PointCloud2::ConstSharedPtr& point_cloud_msg,
                                                 const Image::ConstSharedPtr& rgb_image_msg,
                                                 const CameraInfo::ConstSharedPtr& rgb_camera_info_msg,
                                                 const Image::ConstSharedPtr& depth_image_msg)
{
  // This synchronizer callback runs on a separate thread from the behavior callbacks.
  // All operations in it should be thread-safe.
  setOutput(kOutputNamePointCloud, *point_cloud_msg);
  setOutput(kOutputNameRgbImage, *rgb_image_msg);
  setOutput(kOutputNameRgbCameraInfo, *rgb_camera_info_msg);
  setOutput(kOutputNameDepthImage, *depth_image_msg);
  sync_done_ = true;
  unsubscribeTopics();
}

BT::NodeStatus GetSyncedImagePointCloudDepth::onRunning()
{
  if (sync_done_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  auto node_shrd_ptr = node_.lock();
  if (node_shrd_ptr)
  {
    if (node_shrd_ptr->now() - start_time_ > rclcpp::Duration(kMessageSyncTimeout))
    {
      unsubscribeTopics();
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    unsubscribeTopics();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GetSyncedImagePointCloudDepth::onHalted()
{ unsubscribeTopics(); }

}  // namespace color_tools_behavior
