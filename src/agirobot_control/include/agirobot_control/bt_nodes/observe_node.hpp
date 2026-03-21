#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <memory>
#include "agirobot_control/robot_adapter.hpp"
#include <chrono>

class ObserveNode : public BT::StatefulActionNode
{
public:
  ObserveNode(const std::string& name,
              const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("timeout_ms"),
      BT::OutputPort<std::string>("image_path")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<RobotAdapter> get_adapter();

  bool done_{false};
  std::string image_path_;

  std::chrono::steady_clock::time_point start_time_;
  int timeout_ms_{8000};   // 默认8秒
};