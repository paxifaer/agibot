#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include "agirobot_control/bt_nodes/navigate_node.hpp"
#include "agirobot_control/bt_nodes/observe_node.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("agirobot_bt_runner");

  std::shared_ptr<RobotAdapter> adapter = std::make_shared<TurtleBot3Adapter>(node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateNode>("Navigate");
  factory.registerNodeType<ObserveNode>("Observe");


  auto blackboard = BT::Blackboard::create();
  blackboard->set("adapter", adapter);


  std::string xml_path = 
    std::string(argv[1] ? argv[1] : "behavior_tree.xml");

  auto tree = factory.createTreeFromFile(xml_path, blackboard);

  RCLCPP_INFO(node->get_logger(), "Starting Behavior Tree...");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  std::thread bt_thread([&](){
    while(rclcpp::ok()) {
      tree.tickRoot();

      if(tree.rootNode()->status() == BT::NodeStatus::SUCCESS ||
         tree.rootNode()->status() == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status %d", (int)tree.rootNode()->status());
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });

  exec.spin(); // will return when rclcpp::shutdown is called

  bt_thread.join();
  rclcpp::shutdown();
  return 0;
}