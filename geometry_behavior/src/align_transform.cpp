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
