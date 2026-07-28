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

#include "yolo_behavior/detect_pose.hpp"

namespace yolo_behavior
{

DetectPose::DetectPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList DetectPose::providedPorts()
{
  return {
    // Yolo Config
    BT::InputPort<std::string>(kModelPath, "/path/to/model", "Path to model."),
    BT::InputPort<std::string>(kLabelsPath, "/path/to/label", "Path to label."),

    BT::InputPort<bool>(kUseGpu, true, "Attempt to use GPU"),
    BT::InputPort<float>(kConfThreshold, 0.45, "Confidence Threshold."),
    BT::InputPort<float>(kNmsThreshold, 0.45, "NMS Threshold"),
    BT::InputPort<bool>(kPublishDebugImage, true, "Publish debug image."),

    BT::InputPort<std::string>(kImageTopic, "image_topic", "image topic."),
    BT::InputPort<std::string>(kDebugImageTopic, "~/debug_image", "image topic."),

  };
}

BT::NodeStatus DetectPose::onStart()
{
  ros2_yolos_cpp::YolosConfig c;

  getInput<std::string>(kModelPath, c.model_path);
  getInput<std::string>(kLabelsPath, c.labels_path);

  getInput<bool>(kUseGpu, c.use_gpu);

  getInput<float>(kConfThreshold, conf_);
  getInput<float>(kNmsThreshold, nms_);
  c.conf_threshold = conf_;
  c.nms_threshold = nms_;

  pose_ = std::make_unique<ros2_yolos_cpp::PoseAdapter>();
  if (!pose_->initialize(c))
  {
    return BT::NodeStatus::FAILURE;
  }

  // Start subscribers and synchronizer.
  auto context = this->config().blackboard->get<std::shared_ptr<drt_behavior::DRTTreeContext>>("@drt_context");
  node_ = context->node;
  auto node_raw_ptr = context->node.get();

  std::string image_topic;
  getInput<std::string>(kImageTopic, image_topic);
  image_subscriber = node_raw_ptr->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 1, std::bind(&DetectPose::imageCB, this, std::placeholders::_1));

  getInput<bool>(kPublishDebugImage, publish_debug_image);
  getInput<std::string>(kDebugImageTopic, debug_image_topic);

  if (publish_debug_image)
  {
    debug_publisher = node_raw_ptr->create_publisher<sensor_msgs::msg::Image>(debug_image_topic, 1);
  }

  return BT::NodeStatus::RUNNING;
}

void DetectPose::unsubscribeTopics()
{
  image_subscriber.reset();
  debug_publisher.reset();
  // sub_point_cloud_.unsubscribe();
  // sub_rgb_image_.unsubscribe();
  // sub_rgb_camera_info_.unsubscribe();
  // sub_depth_image_.unsubscribe();
}

void DetectPose::imageCB(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!pose_ || !pose_->isInitialized()) {}
  else
  {
    cv::Mat d;
    auto cv = cv_bridge::toCvShare(msg, "bgr8");
    std::vector<ros2_yolos_cpp::PoseResult> poses = pose_->detect(cv->image, conf_, nms_);
    if (poses.size())
    {
      std::cout << "Pose detected" << std::endl;
      // debug_publisher->publish(conversion::toDetection2DArray(poses, msg->header, msg->width, msg->height));
      if (publish_debug_image && debug_publisher)  // && debug_publisher->is_activated())
      {
        d = cv->image.clone();
        pose_->drawPoses(d, poses);
        // debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", d).toImageMsg());
      }
    }
    else
    {
      std::cout << "Pose not detected" << std::endl;
    }
    if (publish_debug_image)
    {
      debug_publisher->publish(*cv_bridge::CvImage(msg->header, "bgr8", d).toImageMsg());
    }
    detection_done = true;
    unsubscribeTopics();
  }
}

BT::NodeStatus DetectPose::onRunning()
{
  while (!detection_done)
  {
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
  // if (sync_done_)
  // {
  //   return BT::NodeStatus::SUCCESS;
  // }

  // auto node_shrd_ptr = node_.lock();
  // if (node_shrd_ptr)
  // {
  //   if (node_shrd_ptr->now() - start_time_ > rclcpp::Duration(kMessageSyncTimeout))
  //   {
  //     unsubscribeTopics();
  //     return BT::NodeStatus::FAILURE;
  //   }
  // }
  // else
  // {
  //   unsubscribeTopics();
  //   return BT::NodeStatus::FAILURE;
  // }

  // return BT::NodeStatus::RUNNING;
}

void DetectPose::onHalted()
{ unsubscribeTopics(); }

}  // namespace yolo_behavior
