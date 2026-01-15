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

    bool is_action_server_ready() {
    return client_->wait_for_action_server(std::chrono::seconds(0));
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
        goal_msg.pose.header.stamp = node_->now();

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        
        // send_goal_options.result_callback = [cb](auto result) {
        //     if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        //         cb({true, "SUCCEEDED"});
        //     } else {
        //         cb({false, "Navigation failed"});
        //     }
        // };
        send_goal_options.result_callback =
            [cb](auto result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    cb({true, ""});  // 成功就直接返回 OK
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    cb({false, "Nav2 aborted"});
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    cb({false, "Nav2 canceled"});
                    break;
                default:
                    cb({false, "Unknown result code"});
                    break;
            }
        };

        send_goal_options.goal_response_callback = [node = node_](auto future){
            auto goal_handle = future.get();
            if (!goal_handle) RCLCPP_ERROR(node->get_logger(), "Goal rejected");
            else RCLCPP_INFO(node->get_logger(), "Goal accepted");
        };
        send_goal_options.feedback_callback = [node = node_](auto, const auto &feedback){
            // feedback->distance_remaining 等字段取决于 action 定义，打印 useful info
            RCLCPP_DEBUG(node->get_logger(), "Feedback received");
        };


        RCLCPP_INFO(node_->get_logger(), "Sending goal: x=%.2f y=%.2f", pose.pose.position.x, pose.pose.position.y);

        // rclcpp::sleep_for(std::chrono::milliseconds(500));
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void observe(std::function<void(TaskResult, std::string)> cb) override {
        // 最小实现：直接返回成功
        cb({true, ""}, "/tmp/dummy.jpg");
    }

    void follow_target(int target_id, std::function<void(TaskResult)> cb) override {
        (void)target_id;
        cb({false, "Not implemented"});
    }

    void stop(std::function<void(TaskResult)> cb) override {
        // TODO: 可实现cmd_vel zero
        cb({true, ""});
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
};
