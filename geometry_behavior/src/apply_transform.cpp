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
           BT::InputPort<std::vector<double> >("applied_transform"),
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

  std::vector<double> applied_transform_vec;
  if (!getInput("applied_transform", applied_transform_vec))
  {
    throw BT::RuntimeError("Could not access blackboard input [applied_transform]");
  }

  Eigen::Vector3d translation(applied_transform_vec[0], applied_transform_vec[1], applied_transform_vec[2]);
  Eigen::Matrix3d rotation;

  if (applied_transform_vec.size() == 7)
  {
    // quat
    rotation = Eigen::Quaterniond(applied_transform_vec[6], applied_transform_vec[3], applied_transform_vec[4],
                                  applied_transform_vec[5])
                   .toRotationMatrix();
  }
  else if (applied_transform_vec.size() == 6)
  {
    // rpy
    rotation = Eigen::AngleAxisd(applied_transform_vec[5], Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(applied_transform_vec[4], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(applied_transform_vec[3], Eigen::Vector3d::UnitX());
  }
  else
  {
    throw BT::RuntimeError("applied_transform blackboard input is not of the right size");
  }

  Eigen::Isometry3d applied_transform;
  applied_transform.translation() = translation;
  applied_transform.linear() = rotation;

  Eigen::Isometry3d eigen_transform = tf2::transformToEigen(t_stamped);
  Eigen::Isometry3d resulting_transform = eigen_transform * applied_transform;

  // Eigen::Matrix3d rotation_matrix = eigen_transform.rotation();

  // Eigen::Vector3d x_axis = rotation_matrix(Eigen::all, 0);
  // x_axis(2) = 0.0;
  // x_axis.normalize();
  // Eigen::Vector3d z_axis = Eigen::Vector3d(0.0, 0.0, -1.0);
  // Eigen::Vector3d y_axis = z_axis.cross(x_axis);

  // rotation_matrix(Eigen::all, 0) = x_axis;
  // rotation_matrix(Eigen::all, 1) = y_axis;
  // rotation_matrix(Eigen::all, Eigen::last) = Eigen::Vector3d(0.0, 0.0, -1.0);

  // Eigen::Quaterniond aligned_quat(rotation_matrix);
  // aligned_quat.normalize();
  // eigen_transform.linear() = aligned_quat.toRotationMatrix();

  geometry_msgs::msg::TransformStamped res = tf2::eigenToTransform(resulting_transform);
  res.header = t_stamped.header;
  res.child_frame_id = t_stamped.child_frame_id;
  setOutput("resulting_transform", res);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behavior
