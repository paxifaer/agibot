#include "rclcpp/rclcpp.hpp"
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
class TaskOrchestrator : public rclcpp::Node {
public:
    TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter);
    void set_adapter(std::shared_ptr<RobotAdapter> adapter) {
        adapter_ = adapter;
    }

    void run_task_sequence(const json &task_sequence);

   private:
    std::vector<json> tasks_;
    void run_next_task();
    size_t current_task_index_{0};
    void execute_task_json(const std::string &json_str);

    std::shared_ptr<RobotAdapter> adapter_;
};