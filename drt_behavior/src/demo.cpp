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
