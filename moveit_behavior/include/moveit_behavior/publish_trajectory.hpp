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
