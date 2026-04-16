#include <gmock/gmock.h>  // Required for matchers
#include <gtest/gtest.h>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_ros2/tree_execution_server.hpp>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree ID="ColorToolsTestTree">
        <Sequence>
            <ColorBlobFind name="sleepA" msec="2000"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

class BehaviorPluginTest : public ::testing::Test
{
public:
  std::shared_ptr<BT::TreeExecutionServer> tree_exec_server;
  rclcpp::executors::SingleThreadedExecutor executor;

protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    tree_exec_server = std::make_shared<BT::TreeExecutionServer>(options);
    tree_exec_server->node()->set_parameters(
        std::vector<rclcpp::Parameter>{ rclcpp::Parameter("plugins", std::vector<std::string>{
                                                                         "color_tools_behavior/bt_plugins",
                                                                     }) });
    executor.add_node(tree_exec_server->node());
  }

  void TearDown() override
  { tree_exec_server.reset(); }
};

/*
Check to make sure that the plugin is installed and can be loaded by the TreeExecutionServer.
Executor needs to be spun a couple of time (from testing ~10 times) before the method that loads plugin is actually triggered.
*/
TEST_F(BehaviorPluginTest, TestBehaviorRegistration)
{
  bool tree_loaded = false;
  for (int i = 0; i < 100; i++)
  {
    try
    {
      tree_exec_server->factory().registerBehaviorTreeFromText(xml_text);
      break;
    }
    catch (const BT::RuntimeError& e)
    {
      executor.spin_some();
    }
  }

  EXPECT_THAT(tree_exec_server->factory().registeredBehaviorTrees(), ::testing::Contains("ColorToolsTestTree"));
}

int main(int argc, char** argv)
{
  rclcpp::init(0, nullptr);

  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
