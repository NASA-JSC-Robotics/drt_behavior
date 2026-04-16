#pragma once

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "drt_behavior/drt_tree_context.hpp"

#include <tl_expected/expected.hpp>

namespace color_tools_behavior
{

/**
 * @brief Get data from time-synchronized image, depth image, and point cloud topics and populate them in output ports.
 *
 * Slightly modified upstream behavior with an additional port.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                   |
 * | ---------------------------|-----------|-------------------------------|
 * | rgb_camera_info_topic_name | Input     | std::string                   |
 * | rgb_image_topic_name       | Input     | std::string                   |
 * | point_cloud_topic_name     | Input     | std::string                   |
 * | depth_image_topic_name     | Input     | std::string                   |
 * | point_cloud                | Output    | sensor_msgs::msg::PointCloud2 |
 * | depth_image                | Output    | sensor_msgs::msg::Image       |
 * | rgb_image                  | Output    | sensor_msgs::msg::Image       |
 * | rgb_camera_info            | Output    | sensor_msgs::msg::CameraInfo  |
 */
class GetSyncedImagePointCloudDepth : public BT::StatefulActionNode
{
public:
  GetSyncedImagePointCloudDepth(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::weak_ptr<rclcpp::Node> node_;

  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  void syncCallback(const PointCloud2::ConstSharedPtr& point_cloud_msg, const Image::ConstSharedPtr& rgb_image_msg,
                    const CameraInfo::ConstSharedPtr& rgb_camera_info_msg,
                    const Image::ConstSharedPtr& depth_image_msg);
  void unsubscribeTopics();

  message_filters::Subscriber<PointCloud2> sub_point_cloud_;
  image_transport::SubscriberFilter sub_rgb_image_;
  message_filters::Subscriber<CameraInfo> sub_rgb_camera_info_;
  image_transport::SubscriberFilter sub_depth_image_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, Image, CameraInfo, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::shared_ptr<Synchronizer> synchronizer_;

  std::atomic<bool> sync_done_;
  rclcpp::Time start_time_;

  static constexpr std::size_t kMessageSyncQueueSize = 10;
  static constexpr std::chrono::seconds kMessageSyncTimeout{ 5 };

  static constexpr auto kTopicNamePointCloud = "point_cloud_topic_name";
  static constexpr auto kTopicNameRgbImage = "rgb_image_topic_name";
  static constexpr auto kTopicNameRgbCameraInfo = "rgb_camera_info_topic_name";
  static constexpr auto kTopicNameDepthImage = "depth_image_topic_name";

  static constexpr auto kOutputNamePointCloud = "point_cloud";
  static constexpr auto kOutputNameRgbImage = "rgb_image";
  static constexpr auto kOutputNameRgbCameraInfo = "rgb_camera_info";
  static constexpr auto kOutputNameDepthImage = "depth_image";

  static constexpr auto kDescriptionGetSyncedImagePointCloudDepth = R"(
              <p>
                  Get data from time-synchronized image, depth image, and point cloud topics and populate them in output ports.
              </p>
          )";

};  // class GetSyncedImagePointCloudDepth

}  // namespace color_tools_behavior
