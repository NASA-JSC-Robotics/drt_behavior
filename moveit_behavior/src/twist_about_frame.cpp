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

#include "moveit_behavior/twist_about_frame.hpp"

namespace moveit_behavior
{
TwistAboutFrame::TwistAboutFrame(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ThreadedAction(name, config)
{
}

BT::PortsList TwistAboutFrame::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::TransformStamped>("rotation_tf", "TF to rotate about, with respect to world"),
    BT::InputPort<geometry_msgs::msg::TransformStamped>("ee_tf", "TF that will be rotated  with respect to world, usually the End Effector"),
    BT::InputPort<double>("rotation_amount", "Amount of rotation in radians"),
    BT::InputPort<std::string>("rotation_axis", "Choose: X/Y/Z, axis to rotate about the rotation TF"),
    BT::InputPort<bool>("keep_start_orientation", "Keep orientation of EE static throughout rotation"),
    BT::OutputPort<geometry_msgs::msg::TransformStamped>("target_tf")
  };
}

BT::NodeStatus TwistAboutFrame::tick()
{
  geometry_msgs::msg::TransformStamped rotation_tf;
  if (!getInput<geometry_msgs::msg::TransformStamped>("rotation_tf", rotation_tf))
  {
    throw BT::RuntimeError("Could not access blackboard input [rotation_tf]");
  }
  geometry_msgs::msg::TransformStamped ee_tf;
  if (!getInput<geometry_msgs::msg::TransformStamped>("ee_tf", ee_tf))
  {
    throw BT::RuntimeError("Could not access blackboard input [ee_tf]");
  }
  double rotation_amount;
  if (!getInput<double>("rotation_amount", rotation_amount))
  {
    throw BT::RuntimeError("Could not access blackboard input [rotation_amount]");
  }
  std::string rotation_axis;
  if (!getInput<std::string>("rotation_axis", rotation_axis))
  {
    throw BT::RuntimeError("Could not access blackboard input [rotation_axis]");
  }
  bool keep_start_orientation;
  if (!getInput<bool>("keep_start_orientation", keep_start_orientation))
  {
    throw BT::RuntimeError("Could not access blackboard input [keep_start_orientation]");
  }
  
  geometry_msgs::msg::PoseStamped rotation_pose, ee_pose;

  rotation_pose.header = rotation_tf.header;
  rotation_pose.pose.position.x = rotation_tf.transform.translation.x;
  rotation_pose.pose.position.y = rotation_tf.transform.translation.y;
  rotation_pose.pose.position.z = rotation_tf.transform.translation.z;
  rotation_pose.pose.orientation = rotation_tf.transform.rotation;

  ee_pose.header = ee_tf.header;
  ee_pose.pose.position.x = ee_tf.transform.translation.x;
  ee_pose.pose.position.y = ee_tf.transform.translation.y;
  ee_pose.pose.position.z = ee_tf.transform.translation.z;
  ee_pose.pose.orientation = ee_tf.transform.rotation;


  geometry_msgs::msg::PoseStamped new_frame = dex_ivr::rotate_about_frame(rotation_tf.child_frame_id,
                                                                            rotation_pose,
                                                                            ee_pose,
                                                                            rotation_amount, 
                                                                            rotation_axis,
                                                                            keep_start_orientation);

  geometry_msgs::msg::TransformStamped result;
  result.header = new_frame.header;
  result.child_frame_id = "twist_about_frame";
  result.transform.translation.x = new_frame.pose.position.x;
  result.transform.translation.y = new_frame.pose.position.y;
  result.transform.translation.z = new_frame.pose.position.z;
  result.transform.rotation = new_frame.pose.orientation;

  setOutput("target_tf", result);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace moveit_behavior
