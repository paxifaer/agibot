#include "agirobot_control/bt_nodes/observe_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <fstream>
#include <mutex>

static std::mutex obs_log_mtx;

BT::NodeStatus ObserveNode::onStart()
{
  done_ = false;
  image_path_.clear();

  getInput("timeout_ms", timeout_ms_);

  start_time_ = std::chrono::steady_clock::now();

  auto adapter = get_adapter();

  adapter->observe(
    [this](TaskResult res, std::string path) {
      if(res.success) {
        image_path_ = path;
      }
      done_ = true;
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObserveNode::onRunning()
{
  if (done_) {

    auto end = std::chrono::steady_clock::now();
    long long latency_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end - start_time_).count();

    {
      std::lock_guard<std::mutex> lk(obs_log_mtx);
      std::ofstream csv("observe_latency.csv", std::ios::app);
      if (csv.tellp() == 0) {
        csv << "start_ns,end_ns,latency_ms,path\n";
      }
      csv << start_time_.time_since_epoch().count() << ","
          << end.time_since_epoch().count() << ","
          << latency_ms << ","
          << image_path_ << "\n";
    }

    if (!image_path_.empty()) {
      setOutput("image_path", image_path_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - start_time_).count();

  if (elapsed > timeout_ms_) {
    RCLCPP_WARN(rclcpp::get_logger("ObserveNode"),
                "Observe timeout (%d ms)", timeout_ms_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void ObserveNode::onHalted()
{
  RCLCPP_WARN(rclcpp::get_logger("ObserveNode"), "Observe halted");

  done_ = true;   // 防止悬挂状态
}

std::shared_ptr<RobotAdapter> ObserveNode::get_adapter()
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