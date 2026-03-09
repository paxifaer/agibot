#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <memory>
#include "agirobot_control/robot_adapter.hpp"

class ObserveNode : public BT::SyncActionNode
{
public:
  ObserveNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

static BT::PortsList providedPorts()
{
  return {
    BT::InputPort<int>("timeout_ms"),
    BT::OutputPort<std::string>("image_path")
  };
}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<RobotAdapter> get_adapter();
};