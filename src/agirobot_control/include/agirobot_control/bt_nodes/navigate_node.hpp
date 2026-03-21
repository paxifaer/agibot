#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <memory>
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
class NavigateNode : public BT::StatefulActionNode
{
public:
  NavigateNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

static BT::PortsList providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<int>("timeout_ms")
  };
}

  // BT::NodeStatus tick() override;
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  bool done_{false};
  bool success_{false};
  std::shared_ptr<RobotAdapter> get_adapter();
  std::chrono::steady_clock::time_point start_time_;
};