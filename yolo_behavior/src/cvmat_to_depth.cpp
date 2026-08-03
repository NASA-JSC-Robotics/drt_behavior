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

#include "yolo_behavior/cvmat_to_depth.hpp"

namespace yolo_behavior
{

CVMatToDepth::CVMatToDepth(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList CVMatToDepth::providedPorts()
{
  return {
    BT::InputPort<double>(kSubTimeout, 0.5, "N/A"),
    BT::InputPort<std::vector<ros2_yolos_cpp::SegmentationResult>>(kSegmentationResult, "{segmentation_result}",
                                                                   "Depth image"),
    BT::InputPort<std::string>(kDepthTopic, "depth_topic", "image topic."),
    BT::OutputPort<sensor_msgs::msg::Image::SharedPtr>(kDepthImage, "{depth_image}", "image topic."),
  };
}

BT::NodeStatus CVMatToDepth::onStart()
{
  getInput<double>(kSubTimeout, subscription_timeout);

  getInput<std::vector<ros2_yolos_cpp::SegmentationResult>>(kSegmentationResult, segmentation_result);

  // Start subscribers and synchronizer.
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  node_ = context->node;
  auto node_raw_ptr = context->node.get();

  std::string depth_topic;
  getInput<std::string>(kDepthTopic, depth_topic);
  depth_subscriber = node_raw_ptr->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 1, std::bind(&CVMatToDepth::imageCB, this, std::placeholders::_1));

  start_time = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

void CVMatToDepth::unsubscribeTopics()
{ depth_subscriber.reset(); }

void CVMatToDepth::imageCB(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    auto cv = cv_bridge::toCvShare(msg, "16UC1");
    cv::Mat masked_depth = cv::Mat::zeros(cv->image.size(), cv->image.type());

    cv->image.copyTo(masked_depth, segmentation_result[0].mask);

    cv::Scalar average = cv::mean(cv->image, segmentation_result[0].mask);
    std::cout << "DEPTHS AVERAGE : " << average[0] / 1000.0f << std::endl;

    // 2. Initialize the ROS 2 Image message pointer
    sensor_msgs::msg::Image::SharedPtr masked_img;
    masked_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1, masked_depth).toImageMsg();
    setOutput(kDepthImage, masked_img);
  }
  catch (cv_bridge::Exception& e)
  {
    std::cout << "cv_bridge exception" << std::endl;
    // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }

  done_ = true;
  unsubscribeTopics();
}

BT::NodeStatus CVMatToDepth::onRunning()
{
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  std::cout << elapsed << " " << subscription_timeout << std::endl;
  if (elapsed < subscription_timeout)
  {
    if (done_)
    {
      std::cout << "SUCCESS " << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::cout << "RUNNING" << std::endl;
      return BT::NodeStatus::RUNNING;
    }
  }
  else
  {
    std::cout << "FALURE" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

void CVMatToDepth::onHalted()
{ unsubscribeTopics(); }

}  // namespace yolo_behavior
