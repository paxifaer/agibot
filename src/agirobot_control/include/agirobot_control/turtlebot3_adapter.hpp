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
#include <string>
#include <functional>

class TurtleBot3Adapter : public RobotAdapter {
public:
  // 接受两个 node：control_node 用于 action client；perception_node 用于 image subscription
  TurtleBot3Adapter(const rclcpp::Node::SharedPtr &control_node,
                    const rclcpp::Node::SharedPtr &perception_node);

  bool is_action_server_ready();

  void navigate_to(const geometry_msgs::msg::PoseStamped &pose,
                   std::function<void(TaskResult)> cb) override;

  void observe(std::function<void(TaskResult, std::string)> cb) override;

  void follow_target(int target_id, std::function<void(TaskResult)> cb) override;
  void stop(std::function<void(TaskResult)> cb) override;

private:
  // image callback run on perception_node's executor
  void internal_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Node::SharedPtr control_node_;
  rclcpp::Node::SharedPtr perception_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  // subscription held as member so it lives until we reset it
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr tmp_image_sub_;

  std::mutex img_mutex_;
  std::condition_variable img_cv_;
  bool image_received_{false};
  cv::Mat last_image_;
};