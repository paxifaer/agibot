#include "agirobot_control/bt_nodes/navigate_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <fstream>
#include <mutex>

static std::mutex nav_log_mtx;

BT::NodeStatus NavigateNode::tick()
{
  // This implementation keeps simple "start->async callback -> wait" but
  // records timestamps to a CSV file for performance analysis.
  double x, y;
  int timeout_ms = 45000;

  if(!getInput("x", x) || !getInput("y", y)) {
    throw BT::RuntimeError("NavigateNode missing required input x/y");
  }
  getInput("timeout_ms", timeout_ms);

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

  // record start time
  auto start = std::chrono::steady_clock::now();

  std::promise<bool> prom;
  auto fut = prom.get_future();

  adapter->navigate_to(pose,
      [&prom](TaskResult r) {
          prom.set_value(r.success);
  });

  if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
    bool ok = fut.get();

    auto end = std::chrono::steady_clock::now();
    long long latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // write CSV line
    {
      std::lock_guard<std::mutex> lk(nav_log_mtx);
      std::ofstream csv("nav_latency.csv", std::ios::app);
      if (csv.tellp() == 0) {
        csv << "start_ns,end_ns,latency_ms\n";
      }
      csv << start.time_since_epoch().count() << ","
          << end.time_since_epoch().count() << ","
          << latency_ms << "\n";
    }

    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("NavigateNode"), "Navigate node timeout");
    return BT::NodeStatus::FAILURE;
  }
}

std::shared_ptr<RobotAdapter> NavigateNode::get_adapter()
{
    auto bb = config().blackboard;

    if (!bb)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateNode"), "Blackboard is null");
        return nullptr;
    }

    try
    {
        return bb->get<std::shared_ptr<RobotAdapter>>("adapter");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("NavigateNode"),
            "Failed to get adapter: %s",
            e.what());
        return nullptr;
    }
}