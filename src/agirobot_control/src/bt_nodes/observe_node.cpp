#include "agirobot_control/bt_nodes/observe_node.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <fstream>
#include <mutex>

static std::mutex obs_log_mtx;

BT::NodeStatus ObserveNode::tick()
{
  int timeout_ms = 3000;
  getInput("timeout_ms", timeout_ms);

  auto adapter = get_adapter();
  if (!adapter) {
    RCLCPP_ERROR(rclcpp::get_logger("ObserveNode"), "Adapter missing");
    return BT::NodeStatus::FAILURE;
  }

  auto start = std::chrono::steady_clock::now();

  std::promise<std::string> prom;
  auto fut = prom.get_future();

  adapter->observe(
      [prom_ptr = &prom](TaskResult res, std::string path) {
          if(res.success)
              prom_ptr->set_value(path);
          else
              prom_ptr->set_value("");
  });

  if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) ==
      std::future_status::ready)
  {
    std::string path = fut.get();

    auto end = std::chrono::steady_clock::now();
    long long latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // write CSV
    {
      std::lock_guard<std::mutex> lk(obs_log_mtx);
      std::ofstream csv("observe_latency.csv", std::ios::app);
      if (csv.tellp() == 0) {
        csv << "start_ns,end_ns,latency_ms,path\n";
      }
      csv << start.time_since_epoch().count() << ","
          << end.time_since_epoch().count() << ","
          << latency_ms << ","
          << path << "\n";
    }

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