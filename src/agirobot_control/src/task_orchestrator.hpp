#include "rclcpp/rclcpp.hpp"
#include "agirobot_control/robot_adapter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
using json = nlohmann::json;
class TaskOrchestrator : public rclcpp::Node
{
public:
    explicit TaskOrchestrator();
     
    ~TaskOrchestrator();

    void set_adapter(std::shared_ptr<RobotAdapter> adapter);

    void run_task_sequence(const nlohmann::json &tasks);

private:

    void wait_nav(const geometry_msgs::msg::PoseStamped &pose);

    void wait_observe();

    std::shared_ptr<RobotAdapter> adapter_;

    std::ofstream perf_file_;
};