#include <drt_behavior/drt_behavior_btcpp_executor.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  // initialize node
  rclcpp::init(argc, argv);

  // multi-threaded executor
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // create custom BT executor
  auto tree_exec = std::make_shared<drt_behavior::DRTBehaviorBtcppExecutor>(executor);

  // add nodes to executor
  executor->add_node(tree_exec->node());

  // spin, wait for requests to execute trees
  executor->spin();

  // shut down
  executor->remove_node(tree_exec->node());
  rclcpp::shutdown();

  return 0;
}
