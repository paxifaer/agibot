// #include "task_orchestrator.hpp"
// class TaskOrchestrator : public rclcpp::Node {
// public:
//     TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
//     : Node("task_orchestrator"), adapter_(adapter) {}

//     void run_demo() {
//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.frame_id = "odom";
//         // pose.header.stamp = node_->now();
//         pose.pose.position.x = 1.0;
//         pose.pose.position.y = 0.0;
//         pose.pose.orientation.w = 1.0;

         
//         adapter_->navigate_to(pose, [](TaskResult res){
//             rclcpp::sleep_for(std::chrono::milliseconds(500));
//             if(res.success) {
//                 RCLCPP_INFO(rclcpp::get_logger("TaskOrchestrator"), "Navigation success!");
//             } else {
//                 RCLCPP_WARN(rclcpp::get_logger("TaskOrchestrator"), "Navigation failed: %s", res.reason.c_str());
//             }
//         });
//     }
//    private:
//     std::shared_ptr<RobotAdapter> adapter_;
// };

#include "task_orchestrator.hpp"
#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

TaskOrchestrator::TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
: rclcpp::Node("task_orchestrator"),
adapter_(adapter) {}

void TaskOrchestrator::execute_task_json(const std::string &json_str)
{
    auto j = json::parse(json_str);

    for (auto &task : j["tasks"]) {
        std::string type = task["type"];

        if (type == "navigate") {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = task["x"];
            pose.pose.position.y = task["y"];
            pose.pose.orientation.w = 1.0;

            adapter_->navigate_to(pose, [](TaskResult res){
                if (res.success) {
                    RCLCPP_INFO(rclcpp::get_logger("Orchestrator"), "Navigate OK");
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("Orchestrator"),
                                 "Navigate failed: %s", res.reason.c_str());
                }
            });
        }
    }
}


// // main 中
// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("task_orchestrator");

//     auto adapter = std::make_shared<TurtleBot3Adapter>(node);
//     auto orchestrator = std::make_shared<TaskOrchestrator>(adapter);

//     orchestrator->run_demo(); // 调用导航
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_orchestrator_node");

    auto adapter = std::make_shared<TurtleBot3Adapter>(node);
    auto orchestrator = std::make_shared<TaskOrchestrator>(adapter);

    std::string demo_json = R"(
    {
      "tasks": [
        { "type": "navigate", "x": 1.0, "y": 0.0 }
      ]
    })";

    orchestrator->execute_task_json(demo_json);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
