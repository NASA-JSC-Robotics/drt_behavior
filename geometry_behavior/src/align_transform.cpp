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

#include "geometry_behavior/align_transform.hpp"

namespace geometry_behavior
{

AlignTransform::AlignTransform(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
}

BT::PortsList AlignTransform::providedPorts()
{
  return { // output params
           BT::InputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("aligned_transform")
  };
}

BT::NodeStatus AlignTransform::tick()
{
  geometry_msgs::msg::TransformStamped t_stamped;
  if (!getInput("transform_stamped", t_stamped))
  {
    throw BT::RuntimeError("Could not access blackboard input [transform_stamped]");
  }

  Eigen::Isometry3d eigen_transform = tf2::transformToEigen(t_stamped);
  Eigen::Matrix3d rotation_matrix = eigen_transform.rotation();

  Eigen::Vector3d x_axis = rotation_matrix(Eigen::all, 0);
  x_axis(2) = 0.0;
  x_axis.normalize();
  Eigen::Vector3d z_axis = Eigen::Vector3d(0.0, 0.0, -1.0);
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);

  rotation_matrix(Eigen::all, 0) = x_axis;
  rotation_matrix(Eigen::all, 1) = y_axis;
  rotation_matrix(Eigen::all, Eigen::last) = Eigen::Vector3d(0.0, 0.0, -1.0);

  Eigen::Quaterniond aligned_quat(rotation_matrix);
  aligned_quat.normalize();
  eigen_transform.linear() = aligned_quat.toRotationMatrix();

  geometry_msgs::msg::TransformStamped aligned_transform = tf2::eigenToTransform(eigen_transform);
  aligned_transform.header = t_stamped.header;
  setOutput("aligned_transform", aligned_transform);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
