#include "rclcpp/rclcpp.hpp"
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

class TaskOrchestrator : public rclcpp::Node {
public:
    TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter);
    void set_adapter(std::shared_ptr<RobotAdapter> adapter) {
        adapter_ = adapter;
    }
    void execute_task_json(const std::string &json_str);

   private:

    std::shared_ptr<RobotAdapter> adapter_;
};