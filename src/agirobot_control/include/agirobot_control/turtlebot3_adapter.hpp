#pragma once

#include "robot_adapter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <mutex>
#include <condition_variable>

class TurtleBot3Adapter : public RobotAdapter {
public:
    explicit TurtleBot3Adapter(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");

        // tmp_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        //     "/camera/image_raw",
        //     rclcpp::SensorDataQoS(),
        //     std::bind(&TurtleBot3Adapter::image_cb, this, std::placeholders::_1)
        // );
    }

    bool is_action_server_ready() {
        return client_->wait_for_action_server(std::chrono::seconds(0));
    }

    void navigate_to(const geometry_msgs::msg::PoseStamped &pose,
                     std::function<void(TaskResult)> cb) override
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            cb({false, "Nav2 action server not available"});
            return;
        }

        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose = pose;
        goal.pose.header.stamp = node_->now();

        auto options =
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        options.goal_response_callback =
            [node = node_](auto future) {
                if (!future.get()) {
                    RCLCPP_ERROR(node->get_logger(), "Nav goal rejected");
                } else {
                    RCLCPP_INFO(node->get_logger(), "Nav goal accepted");
                }
            };

        options.feedback_callback =
            [node = node_](auto, auto) {
                RCLCPP_DEBUG(node->get_logger(), "Nav feedback");
            };

        options.result_callback =
            [cb](auto result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        cb({true, ""});
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        cb({false, "Nav aborted"});
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        cb({false, "Nav canceled"});
                        break;
                    default:
                        cb({false, "Unknown nav result"});
                        break;
                }
            };

        RCLCPP_INFO(node_->get_logger(),
                    "Navigate to (%.2f, %.2f)",
                    pose.pose.position.x,
                    pose.pose.position.y);

        client_->async_send_goal(goal, options);
    }

void observe(std::function<void(TaskResult, std::string)> cb) override
{
    // 保存订阅到成员变量，保证不会立即销毁
    tmp_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        rclcpp::SensorDataQoS(),
        [this, cb](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                cv::Mat img = cv_ptr->image.clone();

                std::string path =
                    "/tmp/observe_" +
                    std::to_string(node_->now().nanoseconds()) + ".png";
                cv::imwrite(path, img);

                cb({true, ""}, path);

                // 回调完成后释放订阅
                tmp_image_sub_.reset();
            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "cv_bridge error: %s", e.what());
                cb({false, e.what()}, "");
                tmp_image_sub_.reset();
            }
        });
}


// 保证 Subscription 在回调触发前一直存在（用类成员变量保存）

// 回调内部处理图像并在完成后释放订阅

// 不阻塞主线程，全部异步操作

    void follow_target(int, std::function<void(TaskResult)> cb) override {
        cb({false, "Not implemented"});
    }

    void stop(std::function<void(TaskResult)> cb) override {
        cb({true, ""});
    }

private:
    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (image_received_) return;

        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_image_ = cv_ptr->image.clone();
            image_received_ = true;
            img_cv_.notify_one();
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
// 1. TurtleBot3Adapter 里加一个成员变量
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> tmp_image_sub_;

    std::mutex img_mutex_;
    std::condition_variable img_cv_;
    bool image_received_{false};
    cv::Mat last_image_;
};
