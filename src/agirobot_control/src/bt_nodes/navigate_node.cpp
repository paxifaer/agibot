#include "agirobot_control/bt_nodes/navigate_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>

BT::NodeStatus NavigateNode::tick()
{
  double x, y;
  int timeout_ms = 45000;

  if(!getInput("x", x) || !getInput("y", y) || !getInput("timeout_ms", timeout_ms)) {
    throw BT::RuntimeError("NavigateNode missing required input");
  }

  auto adapter = get_adapter();
  if (!adapter) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateNode"), "Adapter not found on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;

  auto prom = std::make_shared<std::promise<bool>>();
  auto fut = prom->get_future();

  adapter->navigate_to(pose,
      [prom](TaskResult r) {
          prom->set_value(r.success);
  });
  if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
    bool ok = fut.get();
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("NavigateNode"), "Navigate node timeout");
    return BT::NodeStatus::FAILURE;
  }
}

std::shared_ptr<RobotAdapter> NavigateNode::get_adapter()
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