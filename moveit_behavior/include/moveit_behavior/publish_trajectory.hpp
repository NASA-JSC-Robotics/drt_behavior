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

#include <moveit_msgs/msg/display_trajectory.hpp>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"

namespace moveit_behavior
{
using DisplayTraj = moveit_msgs::msg::DisplayTrajectory;
/**
 * @brief Converts robot trajectory into display robot trajectory message to be visualized by RViz.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | topic_name                 | Input     | std::string                        |
 * | trajectory                 | Input     | moveit_msgs::msg::RobotTrajectory  |
 */
class PublishDisplayTrajectory : public BT::RosTopicPubNode<DisplayTraj>
{
public:
  explicit PublishDisplayTrajectory(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosTopicPubNode<DisplayTraj>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "moveit trajectory"),
    });
  }

  bool setMessage(DisplayTraj& msg) override;
};
};  // namespace moveit_behavior
