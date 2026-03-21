#include "agirobot_control/bt_nodes/navigate_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <fstream>
#include <mutex>

static std::mutex nav_log_mtx;

BT::NodeStatus NavigateNode::onStart()
{
  double x, y;
  int timeout_ms = 45000;
  start_time_ = std::chrono::steady_clock::now();
  if(!getInput("x", x) || !getInput("y", y)) {
    throw BT::RuntimeError("NavigateNode missing x/y");
  }

  getInput("timeout_ms", timeout_ms);

  auto adapter = get_adapter();
  if (!adapter) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateNode"), "Adapter missing");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;

  done_ = false;
  success_ = false;

  adapter->navigate_to(pose,
      [this](TaskResult r) {
          success_ = r.success;
          done_ = true;
      });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateNode::onRunning()
{
  if (!done_)
    return BT::NodeStatus::RUNNING;

  auto end = std::chrono::steady_clock::now();
  long long latency_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          end - start_time_).count();

  // 写 CSV
  {
    std::lock_guard<std::mutex> lk(nav_log_mtx);
    std::ofstream csv("nav_latency.csv", std::ios::app);
    if (csv.tellp() == 0) {
      csv << "start_ns,end_ns,latency_ms\n";
    }
    csv << start_time_.time_since_epoch().count() << ","
        << end.time_since_epoch().count() << ","
        << latency_ms << "\n";
  }

  return success_ ? BT::NodeStatus::SUCCESS
                  : BT::NodeStatus::FAILURE;
}

void NavigateNode::onHalted()
{
  RCLCPP_WARN(rclcpp::get_logger("NavigateNode"), "Navigation halted");
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