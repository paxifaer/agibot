#pragma once

#include <memory>
#include <string>

#include "robot_adapter.hpp"
#include "turtlebot3_adapter.hpp"
#include "tiago_adapter.hpp"

class RobotAdapterFactory
{
public:

static std::shared_ptr<RobotAdapter> create(
        const std::string &robot,
        rclcpp::Node::SharedPtr control_node,
        rclcpp::Node::SharedPtr perception_node
)
{

    if(robot == "turtlebot3")
    {
        return std::make_shared<TurtleBot3Adapter>(
            control_node,
            perception_node
        );
    }

    if(robot == "tiago")
    {
        return std::make_shared<TiagoAdapter>(
            control_node,
            perception_node
        );
    }

    throw std::runtime_error("Unknown robot type: " + robot);
}

};