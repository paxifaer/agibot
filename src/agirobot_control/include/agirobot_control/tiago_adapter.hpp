#pragma once

#include "agirobot_control/robot_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

class TiagoAdapter : public RobotAdapter
{
public:

    TiagoAdapter(
        rclcpp::Node::SharedPtr control_node,
        rclcpp::Node::SharedPtr perception_node
    );

    void navigate_to(
        const geometry_msgs::msg::PoseStamped &pose,
        std::function<void(TaskResult)> cb
    ) override;

    void observe(
        std::function<void(TaskResult,std::string)> cb
    ) override;

    void follow_target(
        int target_id,
        std::function<void(TaskResult)> cb
    ) override;

    void stop(
        std::function<void(TaskResult)> cb
    ) override;

private:

    rclcpp::Node::SharedPtr control_node_;
    rclcpp::Node::SharedPtr perception_node_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};