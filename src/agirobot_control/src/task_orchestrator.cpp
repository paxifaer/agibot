#include "task_orchestrator.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <curl/curl.h>
#include <future>

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

json request_from_llm(const std::string &text) {
    CURL* curl = curl_easy_init();
    std::string buffer;
    json result;

    if (curl) {
        std::string post = "{\"text\":\"" + text + "\"}";
        curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8000/parse");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
        curl_easy_setopt(curl, CURLOPT_PROXY, "");
        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        result = json::parse(buffer);
    }
    return result;
}

TaskOrchestrator::TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
: rclcpp::Node("task_orchestrator",
    rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)),
  adapter_(adapter)
{
}

void TaskOrchestrator::run_task_sequence(const json &tasks)
{
    tasks_.clear();
    for (auto &t : tasks["tasks"]) tasks_.push_back(t);

    current_task_index_ = 0;
    RCLCPP_INFO(get_logger(), "Start task sequence (%zu tasks)", tasks_.size());
    run_next_task();
}

void TaskOrchestrator::run_next_task()
{
    if (current_task_index_ >= tasks_.size()) {
        RCLCPP_INFO(get_logger(), "All tasks finished");
        return;
    }

    const auto &task = tasks_[current_task_index_];
    std::string type = task["type"];

    RCLCPP_INFO(get_logger(), "Task %zu: %s",
                current_task_index_, type.c_str());

    if (type == "navigate") {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = task["pose"]["x"];
        pose.pose.position.y = task["pose"]["y"];
        pose.pose.orientation.w = 1.0;

        adapter_->navigate_to(
            pose,
            [this](TaskResult res) {
                if (!res.success) {
                    RCLCPP_ERROR(get_logger(), "Navigate failed");
                    return;
                }
                current_task_index_++;
                run_next_task();
            }
        );
    }
    else if (type == "observe") {
        adapter_->observe(
            [this](TaskResult res, std::string path) {
                if (!res.success) {
                    RCLCPP_ERROR(get_logger(), "Observe failed");
                    return;
                }
                RCLCPP_INFO(get_logger(), "Image saved: %s", path.c_str());
                current_task_index_++;
                run_next_task();
            }
        );
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto orchestrator =
        std::make_shared<TaskOrchestrator>(nullptr);

    auto adapter =
        std::make_shared<TurtleBot3Adapter>(orchestrator);

    orchestrator->set_adapter(adapter);

    while (!adapter->is_action_server_ready()) {
        RCLCPP_INFO(orchestrator->get_logger(),
                    "Waiting for Nav2 action server...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    json tasks = request_from_llm("去厨房看看桌上有没有红杯子");
    orchestrator->run_task_sequence(tasks);

    rclcpp::spin(orchestrator);
    rclcpp::shutdown();
    return 0;
}
