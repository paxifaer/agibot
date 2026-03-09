#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <memory>
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigateNode : public BT::SyncActionNode
{
public:
  NavigateNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

static BT::PortsList providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<int>("timeout_ms")
  };
}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<RobotAdapter> get_adapter();
};