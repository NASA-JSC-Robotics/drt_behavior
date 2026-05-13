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

#include "geometry_behavior/apply_transform.hpp"

namespace geometry_behavior
{

ApplyTransform::ApplyTransform(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList ApplyTransform::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("input_transform"),
           BT::InputPort<std::vector<double> >("applied_transform_translation", std::vector<double>{ 0.0, 0.0, 0.0 }, "XYZ translation"),
           BT::InputPort<std::vector<double> >("applied_transform_rotation", std::vector<double>{ 0.0, 0.0, 0.0, 1.0 },  "Either RPY in radians, or XYZ-W quaternions"),
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("resulting_transform")
  };
}

// You must override the virtual function tick()
BT::NodeStatus ApplyTransform::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("input_transform", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [input_transform]");
  }

  std::vector<double> applied_transform_translation;
  if (!getInput("applied_transform", applied_transform_translation))
  {
    throw BT::RuntimeError("Could not access blackboard input [applied_transform_translation]");
  }

  std::vector<double> applied_transform_rotation;
  if (!getInput("applied_transform", applied_transform_rotation))
  {
    throw BT::RuntimeError("Could not access blackboard input [applied_transform_rotation]");
  }

  if (applied_transform_translation.size() != 3)
  {
    throw BT::RuntimeError("applied_transform_translation blackboard input is not of the right size");
  }
  Eigen::Vector3d translation(applied_transform_translation[0], applied_transform_translation[1], applied_transform_translation[2]);

  Eigen::Matrix3d rotation;
  if (applied_transform_rotation.size() == 4)
  {
    // quat
    rotation = Eigen::Quaterniond(applied_transform_rotation[3], applied_transform_rotation[0], applied_transform_rotation[1],
                                  applied_transform_rotation[2])
                   .toRotationMatrix();
  }
  else if (applied_transform_rotation.size() == 3)
  {
    // rpy
    rotation = Eigen::AngleAxisd(applied_transform_rotation[2], Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(applied_transform_rotation[1], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(applied_transform_rotation[0], Eigen::Vector3d::UnitX());
  }
  else
  {
    throw BT::RuntimeError("applied_transform_rotation blackboard input is not of the right size");
  }

  Eigen::Isometry3d applied_transform;
  applied_transform.translation() = translation;
  applied_transform.linear() = rotation;

  Eigen::Isometry3d eigen_transform = tf2::transformToEigen(t_stamped);
  Eigen::Isometry3d resulting_transform = eigen_transform * applied_transform;

  geometry_msgs::msg::TransformStamped res = tf2::eigenToTransform(resulting_transform);
  res.header = t_stamped.header;
  res.child_frame_id = t_stamped.child_frame_id;
  setOutput("resulting_transform", res);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
