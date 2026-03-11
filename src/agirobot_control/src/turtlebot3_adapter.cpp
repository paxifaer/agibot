#include "agirobot_control/turtlebot3_adapter.hpp"
#include <chrono>
#include <sstream>

TurtleBot3Adapter::TurtleBot3Adapter(const rclcpp::Node::SharedPtr &control_node,
                                     const rclcpp::Node::SharedPtr &perception_node)
  : control_node_(control_node),
    perception_node_(perception_node)
{
  nav_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            control_node_, "navigate_to_pose");
}

bool TurtleBot3Adapter::is_action_server_ready() {
  return nav_client_->wait_for_action_server(std::chrono::seconds(0));
}

void TurtleBot3Adapter::navigate_to(const geometry_msgs::msg::PoseStamped &pose,
                                   std::function<void(TaskResult)> cb)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    cb({false, "Nav2 action server not available"});
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = pose;
  goal.pose.header.stamp = control_node_->now();

  auto options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  options.goal_response_callback =
      [node = control_node_](auto future) {
        try {
          auto goal_handle = future.get();
          if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Nav goal rejected");
          } else {
            RCLCPP_DEBUG(node->get_logger(), "Nav goal accepted");
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(node->get_logger(), "Goal response exception: %s", e.what());
        }
      };

  options.feedback_callback =
      [node = control_node_](auto, auto) {
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

  RCLCPP_INFO(control_node_->get_logger(),
              "Navigate to (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);

  nav_client_->async_send_goal(goal, options);
}

void TurtleBot3Adapter::observe(std::function<void(TaskResult, std::string)> cb)
{
  // subscribe on perception_node_. Use SensorDataQoS for camera.
  // Keep subscription alive until first frame processed, then reset.

  // prepare callback that captures cb and this
  auto lambda = [this, cb](const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img = cv_ptr->image.clone();

      std::ostringstream ss;
      ss << "/tmp/observe_" << perception_node_->now().nanoseconds() << ".png";
      std::string path = ss.str();

      // write image
      cv::imwrite(path, img);

      // call user callback
      cb({true, ""}, path);

      // unsubscribe (release)
      tmp_image_sub_.reset();
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(perception_node_->get_logger(), "cv_bridge error: %s", e.what());
      cb({false, e.what()}, "");
      tmp_image_sub_.reset();
    }
  };

  tmp_image_sub_ = perception_node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      rclcpp::SensorDataQoS(),
      lambda);
}

void TurtleBot3Adapter::follow_target(int, std::function<void(TaskResult)> cb)
{
  cb({false, "Not implemented"});
}

void TurtleBot3Adapter::stop(std::function<void(TaskResult)> cb)
{
  cb({true, ""});
}