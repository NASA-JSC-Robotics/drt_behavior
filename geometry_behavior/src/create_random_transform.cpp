#include "geometry_behavior/create_random_transform.hpp"

namespace geometry_behavior
{

CreateRandomTransform::CreateRandomTransform(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

// It is mandatory to define this STATIC method.
BT::PortsList CreateRandomTransform::providedPorts()
{
  return { // output params
           BT::InputPort<std::vector<double> >("center", std::vector<double>{ 0.0, 0.0, 0.0 }, "center of the box"),
           BT::InputPort<std::vector<double> >("box_side", std::vector<double>{ 1.0, 1.0, 1.0 }, "sides of the box"),
           BT::InputPort<std::string>("transform_name"),
           BT::OutputPort<geometry_msgs::msg::TransformStamped>("transform")
  };
}

// You must override the virtual function tick()
BT::NodeStatus CreateRandomTransform::tick()
{
  std::string transform_name;
  if (!getInput("transform_name", transform_name))
  {
    throw BT::RuntimeError("Could not access blackboard input [transform_name]");
  }

  std::vector<double> c, bs;
  getInput("center", c);
  getInput("box_side", bs);

  Eigen::Vector3d center(c.data());
  Eigen::Vector3d box_side{ bs.data() };
  Eigen::Vector3d random = Eigen::Vector3d::Random();

  Eigen::Vector3d res = -1.0 * (box_side / 2) + (random.cwiseProduct(box_side)) + center;

  Eigen::Isometry3d resulting_transform(Eigen::Isometry3d::Identity());
  resulting_transform.translation() = res;
  geometry_msgs::msg::TransformStamped t_stamped = tf2::eigenToTransform(resulting_transform);

  t_stamped.header.frame_id = "world";
  t_stamped.child_frame_id = transform_name;

  setOutput("transform", t_stamped);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace geometry_behavior
