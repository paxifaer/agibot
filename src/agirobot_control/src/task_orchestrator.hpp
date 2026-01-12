#include "rclcpp/rclcpp.hpp"
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

class TaskOrchestrator : public rclcpp::Node {
public:
    TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter);
    
    void run_demo() {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        // pose.header.stamp = node_->now();
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 0.0;
        pose.pose.orientation.w = 1.0;

         
        adapter_->navigate_to(pose, [](TaskResult res){
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            if(res.success) {
                RCLCPP_INFO(rclcpp::get_logger("TaskOrchestrator"), "Navigation success!");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("TaskOrchestrator"), "Navigation failed: %s", res.reason.c_str());
            }
        });
    }
    
    void execute_task_json(const std::string &json_str);

   private:

    std::shared_ptr<RobotAdapter> adapter_;
};