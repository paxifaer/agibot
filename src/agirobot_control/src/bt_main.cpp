#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include "agirobot_control/bt_nodes/navigate_node.hpp"
#include "agirobot_control/bt_nodes/observe_node.hpp"
#include "agirobot_control/robot_adapter_factory.hpp"
#include <thread>
#include <chrono>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string robot = "turtlebot3";

  for(int i=0;i<argc;i++)
  {
      if(std::string(argv[i]) == "--robot" && i+1 < argc)
      {
          robot = argv[i+1];
      }
  }
  // control node: for BT and action client
  auto control_node = std::make_shared<rclcpp::Node>("agirobot_control_node");

  // perception node: for camera subscription; spin separately
  auto perception_node = std::make_shared<rclcpp::Node>("agirobot_perception_node");

  auto adapter =
      RobotAdapterFactory::create(
          robot,
          control_node,
          perception_node
      );

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateNode>("Navigate");
  factory.registerNodeType<ObserveNode>("Observe");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("adapter", adapter);

  std::string xml_path =
    std::string(argv[1] ? argv[1] : "behavior_tree.xml");

  auto tree = factory.createTreeFromFile(xml_path, blackboard);

  RCLCPP_INFO(control_node->get_logger(), "Starting Behavior Tree (control executor + perception executor)...");

  // control executor (multi-threaded)
  rclcpp::executors::MultiThreadedExecutor control_exec;
  control_exec.add_node(control_node);

  // perception executor (single-threaded) - dedicated for sensors to avoid contention
  rclcpp::executors::SingleThreadedExecutor perception_exec;
  perception_exec.add_node(perception_node);

  // Spin perception node in a separate thread so camera callbacks are handled independently
  std::thread perception_thread([&]() {
    perception_exec.spin();
  });

  // BT ticking in another thread so control_exec can process callbacks concurrently
  std::atomic_bool stop_flag(false);
  std::thread bt_thread([&]() {
    while (rclcpp::ok() && !stop_flag.load()) {
      tree.tickRoot();
      // if tree finished, break to allow graceful shutdown
      if (tree.rootNode()->status() == BT::NodeStatus::SUCCESS ||
          tree.rootNode()->status() == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(control_node->get_logger(), "Behavior tree finished with status %d",
                    (int)tree.rootNode()->status());
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // signal shutdown
    rclcpp::shutdown();
  });

  // Spin control executor in main thread (blocks until shutdown)
  control_exec.spin();

  // cleanup
  stop_flag.store(true);
  if (bt_thread.joinable()) bt_thread.join();
  if (perception_thread.joinable()) {
    // perception_exec will stop when rclcpp::shutdown called by bt_thread
    perception_thread.join();
  }

  return 0;
}