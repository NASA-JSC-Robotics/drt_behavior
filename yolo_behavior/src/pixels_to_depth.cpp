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

#include "yolo_behavior/pixels_to_depth.hpp"

namespace yolo_behavior
{

PixelsToDepth::PixelsToDepth(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PixelsToDepth::providedPorts()
{
  return {
    BT::InputPort<double>(kSubTimeout, 0.5, "N/A"),
    BT::InputPort<std::vector<ros2_yolos_cpp::PoseResult>>(kPoses, "{poses}", "Depth image"),
    BT::InputPort<std::string>(kDepthTopic, "depth_topic", "image topic."),
  };
}

BT::NodeStatus PixelsToDepth::onStart()
{
  getInput<double>(kSubTimeout, subscription_timeout);

  getInput<std::vector<ros2_yolos_cpp::PoseResult>>(kPoses, poses);

  // Start subscribers and synchronizer.
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  node_ = context->node;
  auto node_raw_ptr = context->node.get();

  std::string depth_topic;
  getInput<std::string>(kDepthTopic, depth_topic);
  depth_subscriber = node_raw_ptr->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 1, std::bind(&PixelsToDepth::imageCB, this, std::placeholders::_1));

  // getInput<bool>(kPublishDebugImage, publish_debug_image);
  // getInput<std::string>(kDebugImageTopic, debug_image_topic);

  // if (publish_debug_image)
  // {
  //   debug_publisher = node_raw_ptr->create_publisher<sensor_msgs::msg::Image>(debug_image_topic, 1);
  // }
  start_time = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

void PixelsToDepth::unsubscribeTopics()
{ depth_subscriber.reset(); }

void PixelsToDepth::imageCB(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert to cv::Mat using passthrough encoding
    auto cv = cv_bridge::toCvShare(msg, "16UC1");

    std::vector<float> depths;

    for (auto keypoint : poses[0].keypoints)
    {
      std::cout << "KEYPOINT CONFIDENCE : " << keypoint.confidence << std::endl;
      int row = static_cast<int>(keypoint.x);
      int col = static_cast<int>(keypoint.y);
      // Handle 16-bit Unsigned Integer Depth Maps (Values in mm)
      if (cv->image.type() == CV_16UC1)
      {
        ushort depth_mm = cv->image.at<ushort>(row, col);
        float depth_m = depth_mm / 1000.0f;  // Convert mm to meters

        std::cout << "Raw 16-bit Depth: " << depth_mm << " mm" << std::endl;
        std::cout << "Physical Depth:   " << depth_m << " m" << std::endl;

        depths.push_back(depth_m);
      }
      // Handle 32-bit Floating-Point Depth Maps (Values in meters)
      else if (cv->image.type() == CV_32FC1)
      {
        float depth_m = cv->image.at<float>(row, col);  // Direct value in meters
        depths.push_back(depth_m);

        std::cout << "Physical Depth: " << depth_m << " m" << std::endl;
      }
      else
      {
        std::cout << "Unsupported image format or not a single-channel depth image." << std::endl;
      }
    }

    float sum = std::accumulate(depths.begin(), depths.end(), 0.0f);
    float average = sum / depths.size();
    std::cout << "DEPTHS AVERAGE : " << average << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    std::cout << "cv_bridge exception" << std::endl;
    // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }

  done_ = true;
  unsubscribeTopics();
}

BT::NodeStatus PixelsToDepth::onRunning()
{
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  if (!done_ && elapsed < subscription_timeout)
  {
    return BT::NodeStatus::RUNNING;
  }
  else if (elapsed >= subscription_timeout)
  {
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    return BT::NodeStatus::SUCCESS;
  }
}

void PixelsToDepth::onHalted()
{ unsubscribeTopics(); }

}  // namespace yolo_behavior
