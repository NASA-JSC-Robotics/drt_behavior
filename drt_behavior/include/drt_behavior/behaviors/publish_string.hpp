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

#include <std_msgs/msg/string.hpp>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"

using StringMsg = std_msgs::msg::String;
/**
 * @brief Fills in `std_msgs::msg::String` message's data field with `data` input port, and publishes it on the `topic_name`.
 *
 * @details
 * | Data Port Name             | Port Type | Object Type                        |
 * | ---------------------------|-----------|------------------------------------|
 * | topic_name                 | Input     | std::string                        |
 * | data                       | Input     | std::string                        |
 */
class PublishString : public BT::RosTopicPubNode<StringMsg>
{
public:
  explicit PublishString(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<StringMsg>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("data")
    });
  }

  bool setMessage(StringMsg& msg) override;
};
