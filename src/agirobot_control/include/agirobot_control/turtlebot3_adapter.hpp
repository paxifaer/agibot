#pragma once
#include "robot_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TurtleBot3Adapter : public RobotAdapter {
public:
    TurtleBot3Adapter(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    void navigate_to(const geometry_msgs::msg::PoseStamped &pose,
                     std::function<void(TaskResult)> cb) override
    {
        if(!client_->wait_for_action_server(std::chrono::seconds(5))) {
            cb({false, "Nav2 action server not available"});
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = pose;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [cb](auto result) {
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                cb({true, ""});
            } else {
                cb({false, "Navigation failed"});
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void observe(std::function<void(TaskResult, std::string)> cb) override {
        // 最小实现：直接返回成功
        cb({true, ""}, "/tmp/dummy.jpg");
    }

    void follow_target(int target_id, std::function<void(TaskResult)> cb) override {
        cb({false, "Not implemented"}, "");
    }

    void stop(std::function<void(TaskResult)> cb) override {
        // TODO: 可实现cmd_vel zero
        cb({true, ""}, "");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
};
