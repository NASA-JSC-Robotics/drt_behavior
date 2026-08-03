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

#include "yolo_behavior/depth_to_pointcloud.hpp"

namespace yolo_behavior
{

DepthToPointcloud::DepthToPointcloud(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList DepthToPointcloud::providedPorts()
{
  return {
    BT::InputPort<double>(kSubTimeout, 0.5, "N/A"),
    // Yolo Config
    BT::InputPort<std::string>(kCameraInfoTopic, "camera_info", "image topic."),
    BT::InputPort<double>(kInvalidDepth, 0.0, "image topic."),
    BT::InputPort<sensor_msgs::msg::Image::SharedPtr>(kDepthImage, "{depth_image}", "image topic."),

    BT::OutputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
  };
}

BT::NodeStatus DepthToPointcloud::onStart()
{
  getInput<double>(kSubTimeout, subscription_timeout);

  getInput<std::string>(kCameraInfoTopic, camera_info_topic);
  getInput<double>(kInvalidDepth, invalid_depth);
  getInput<sensor_msgs::msg::Image::SharedPtr>(kDepthImage, depth_msg);

  // Start subscribers and synchronizer.
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  node_ = context->node;
  auto node_raw_ptr = context->node.get();

  std::string camera_info_topic;
  getInput<std::string>(kCameraInfoTopic, camera_info_topic);
  camera_info_subscriber = node_raw_ptr->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 1, std::bind(&DepthToPointcloud::cameraInfoCB, this, std::placeholders::_1));

  // getInput<bool>(kPublishDebugImage, publish_debug_image);
  // getInput<std::string>(kDebugImageTopic, debug_image_topic);

  if (publish_debug_image)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    debug_publisher = node_raw_ptr->create_publisher<sensor_msgs::msg::Image>(debug_image_topic, qos);
  }
  start_time = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

void DepthToPointcloud::unsubscribeTopics()
{ camera_info_subscriber.reset(); }

void DepthToPointcloud::cameraInfoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::cout << "Got camera info " << std::endl;
  // Update camera model
  image_geometry::PinholeCameraModel model_;
  model_.fromCameraInfo(msg);

  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Convert Depth Image to Pointcloud
  if (depth_msg->encoding == enc::TYPE_16UC1 || depth_msg->encoding == enc::MONO16)
  {
    depth_image_proc::convertDepth<uint16_t>(depth_msg, cloud_msg, model_, invalid_depth);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    depth_image_proc::convertDepth<float>(depth_msg, cloud_msg, model_, invalid_depth);
  }

  // 1. Convert ROS 2 PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // 2. Prepare Eigen variables for output
  Eigen::Vector3f centroid;
  Eigen::Vector3f obb_center;
  Eigen::Vector3f obb_dimensions;
  Eigen::Matrix3f obb_rotation_matrix;

  // 3. Compute centroid and OBB
  pcl::computeCentroidAndOBB(*cloud, centroid, obb_center, obb_dimensions, obb_rotation_matrix);

  geometry_msgs::msg::TransformStamped t_stamped;
  t_stamped.header = depth_msg->header;
  t_stamped.child_frame_id = "estimated_human_pose";

  t_stamped.transform.translation.x = centroid(0);
  t_stamped.transform.translation.y = centroid(1);
  t_stamped.transform.translation.z = centroid(2);
  t_stamped.transform.rotation.w = 1.0;
  setOutput("transform_stamped", t_stamped);

  done_ = true;
  unsubscribeTopics();
}

BT::NodeStatus DepthToPointcloud::onRunning()
{
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  if (elapsed < subscription_timeout)
  {
    if (done_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void DepthToPointcloud::onHalted()
{ unsubscribeTopics(); }

}  // namespace yolo_behavior
