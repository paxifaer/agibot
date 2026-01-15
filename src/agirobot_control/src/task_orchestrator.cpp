#include "task_orchestrator.hpp"
#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;


TaskOrchestrator::TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
: rclcpp::Node("task_orchestrator", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)),
  adapter_(adapter)
{
}


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



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto orchestrator = std::make_shared<TaskOrchestrator>(nullptr);   // no nullptr
  auto adapter = std::make_shared<TurtleBot3Adapter>(orchestrator);
  orchestrator->set_adapter(adapter);

  // wait action server ready
  while (!adapter->is_action_server_ready()) {
    RCLCPP_INFO(orchestrator->get_logger(), "Waiting for Nav2 action server...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

    auto demo_json = R"({
      "tasks": [
        { "type": "navigate", "x": 0.1, "y": 0.0, "yaw": 0.0  }
      ]
    })";

  orchestrator->execute_task_json(demo_json);
  rclcpp::spin(orchestrator);
  rclcpp::shutdown();
  return 0;
}