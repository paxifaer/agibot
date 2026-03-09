#include "agirobot_control/bt_nodes/observe_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>

BT::NodeStatus ObserveNode::tick()
{
  int timeout_ms = 3000; 

  getInput("timeout_ms", timeout_ms);  

  auto adapter = get_adapter();
  if (!adapter) {
    RCLCPP_ERROR(rclcpp::get_logger("ObserveNode"), "Adapter missing");
    return BT::NodeStatus::FAILURE;
  }

  auto prom = std::make_shared<std::promise<std::string>>();
  auto fut = prom->get_future();

  adapter->observe(
      [prom](TaskResult res, std::string path) {
          if(res.success)
              prom->set_value(path);
          else
              prom->set_value("");
  });
  if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) ==
      std::future_status::ready)
  {
    std::string path = fut.get();

    if(!path.empty()) {
      setOutput("image_path", path);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

std::shared_ptr<RobotAdapter> ObserveNode::get_adapter()
{
  auto bb = config().blackboard;

  if(!bb)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateNode"), "Blackboard is null");
    return nullptr;
  }

  try
  {
    auto adapter = bb->get<std::shared_ptr<RobotAdapter>>("adapter");
    return adapter;
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateNode"),
                 "Failed to get adapter: %s", e.what());
    return nullptr;
  }
}