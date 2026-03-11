#include "agirobot_control/tiago_adapter.hpp"

#include <chrono>

using namespace std::chrono_literals;

TiagoAdapter::TiagoAdapter(
    rclcpp::Node::SharedPtr control_node,
    rclcpp::Node::SharedPtr perception_node
)
{
    control_node_ = control_node;
    perception_node_ = perception_node;

    nav_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            control_node_,
            "/navigate_to_pose"
        );
}


void TiagoAdapter::navigate_to(
    const geometry_msgs::msg::PoseStamped &pose,
    std::function<void(TaskResult)> cb
)
{

    if(!nav_client_->wait_for_action_server(5s))
    {
        cb({false,"Nav2 not available"});
        return;
    }

    nav2_msgs::action::NavigateToPose::Goal goal;

    goal.pose = pose;
    goal.pose.header.stamp = control_node_->now();

    auto options =
        rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose
        >::SendGoalOptions();

    options.result_callback =
        [cb](auto result)
        {
            if(result.code ==
               rclcpp_action::ResultCode::SUCCEEDED)
            {
                cb({true,""});
            }
            else
            {
                cb({false,"Navigation failed"});
            }
        };

    RCLCPP_INFO(
        control_node_->get_logger(),
        "Tiago navigate to %.2f %.2f",
        pose.pose.position.x,
        pose.pose.position.y
    );

    nav_client_->async_send_goal(goal,options);
}


void TiagoAdapter::observe(
    std::function<void(TaskResult,std::string)> cb
)
{

    image_sub_ =
        perception_node_->create_subscription<
            sensor_msgs::msg::Image
        >(
            "/xtion/rgb/image_raw",
            rclcpp::SensorDataQoS(),

            [this,cb](const sensor_msgs::msg::Image::SharedPtr msg)
            {

                try
                {
                    auto cv_ptr =
                        cv_bridge::toCvCopy(msg,"bgr8");

                    auto img = cv_ptr->image.clone();

                    std::string path =
                        "/tmp/tiago_observe_" +
                        std::to_string(
                            perception_node_->now().nanoseconds()
                        ) +
                        ".png";

                    cv::imwrite(path,img);

                    cb({true,""},path);

                    image_sub_.reset();
                }

                catch(const cv_bridge::Exception &e)
                {
                    cb({false,e.what()},"");
                }

            }
        );
}


void TiagoAdapter::follow_target(
    int,
    std::function<void(TaskResult)> cb
)
{
    cb({false,"Not implemented"});
}


void TiagoAdapter::stop(
    std::function<void(TaskResult)> cb
)
{
    cb({true,""});
}