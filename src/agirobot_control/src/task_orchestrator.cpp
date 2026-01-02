#include "rclcpp/rclcpp.hpp"
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

class TaskOrchestrator : public rclcpp::Node {
public:
    TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
    : Node("task_orchestrator"), adapter_(adapter) {}

    void run_demo() {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 0.0;
        pose.pose.orientation.w = 1.0;

        adapter_->navigate_to(pose, [](TaskResult res){
            if(res.success) {
                RCLCPP_INFO(rclcpp::get_logger("TaskOrchestrator"), "Navigation success!");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("TaskOrchestrator"), "Navigation failed: %s", res.reason.c_str());
            }
        });
    }

private:
    std::shared_ptr<RobotAdapter> adapter_;
};



// main 中
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_orchestrator");

    auto adapter = std::make_shared<TurtleBot3Adapter>(node);
    auto orchestrator = std::make_shared<TaskOrchestrator>(adapter);

    orchestrator->run_demo(); // 调用导航
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

