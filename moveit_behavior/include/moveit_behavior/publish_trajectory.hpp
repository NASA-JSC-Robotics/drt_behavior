#include <moveit_msgs/msg/display_trajectory.hpp>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"

namespace moveit_behavior
{
using DisplayTraj = moveit_msgs::msg::DisplayTrajectory;

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

  bool setMessage(DisplayTraj& msg) override
  {
    if (!getInput("trajectory", msg.trajectory))
    {
      std::cout << "Could not access required blackboard input [trajectory]" << std::endl;
      return false;
    }
    return true;
  }
  //   BT::NodeStatus onTick(const std::shared_ptr<moveit_msgs::msg::DisplayTrajectory>& last_msg) override;
};
};  // namespace moveit_behavior
