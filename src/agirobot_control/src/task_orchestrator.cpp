#include "task_orchestrator.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <curl/curl.h>
#include <future>
#include <iostream>

using json = nlohmann::json;

/* ============================= */
/*         CURL 回调函数         */
/* ============================= */

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t total = size * nmemb;
    ((std::string*)userp)->append((char*)contents, total);
    return total;
}

/* ============================= */
/*        LLM 请求函数（安全版）  */
/* ============================= */

json request_from_llm(const std::string &text)
{
    CURL* curl = curl_easy_init();
    std::string buffer;
    json result;

    if (!curl) {
        std::cerr << "Failed to init curl" << std::endl;
        return result;
    }

    // 构造 JSON 请求体（防止中文转义问题）
    json req_body;
    req_body["text"] = text;
    std::string post_data = req_body.dump();

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8000/parse");
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
    curl_easy_setopt(curl, CURLOPT_PROXY, "");

    CURLcode res = curl_easy_perform(curl);

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    if (res != CURLE_OK) {
        std::cerr << "curl error: "
                  << curl_easy_strerror(res) << std::endl;
        return result;
    }

    if (http_code != 200) {
        std::cerr << "HTTP error: " << http_code << std::endl;
        std::cerr << "Response: " << buffer << std::endl;
        return result;
    }

    if (buffer.empty()) {
        std::cerr << "Empty response from LLM server" << std::endl;
        return result;
    }

    try {
        result = json::parse(buffer);
    }
    catch (const std::exception &e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        std::cerr << "Raw response: " << buffer << std::endl;
    }

    return result;
}

/* ============================= */
/*        TaskOrchestrator       */
/* ============================= */

TaskOrchestrator::TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
: rclcpp::Node("task_orchestrator",
    rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)),
  adapter_(adapter)
{
}

void TaskOrchestrator::run_task_sequence(const json &tasks)
{
    if (!tasks.contains("tasks") || !tasks["tasks"].is_array()) {
        RCLCPP_ERROR(get_logger(), "Invalid task JSON format!");
        return;
    }

    tasks_.clear();
    for (auto &t : tasks["tasks"])
        tasks_.push_back(t);

    if (tasks_.empty()) {
        RCLCPP_WARN(get_logger(), "No tasks received.");
        return;
    }

    current_task_index_ = 0;

    RCLCPP_INFO(get_logger(),
        "Start task sequence (%zu tasks)", tasks_.size());

    run_next_task();
}

void TaskOrchestrator::run_next_task()
{
    if (current_task_index_ >= tasks_.size()) {
        RCLCPP_INFO(get_logger(), "All tasks finished");
        return;
    }

    const auto &task = tasks_[current_task_index_];

    if (!task.contains("type")) {
        RCLCPP_ERROR(get_logger(), "Task missing 'type' field");
        return;
    }

    std::string type = task["type"];

    RCLCPP_INFO(get_logger(), "Task %zu: %s",
                current_task_index_, type.c_str());

    /* ================= navigate ================= */

    if (type == "navigate") {

        if (!task.contains("pose")) {
            RCLCPP_ERROR(get_logger(), "Navigate task missing pose");
            return;
        }

        auto pose_json = task["pose"];

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();

        pose.pose.position.x = pose_json.value("x", 0.0);
        pose.pose.position.y = pose_json.value("y", 0.0);
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

    /* ================= observe ================= */

    else if (type == "observe") {

        adapter_->observe(
            [this](TaskResult res, std::string path) {
                if (!res.success) {
                    RCLCPP_ERROR(get_logger(), "Observe failed");
                    return;
                }
                RCLCPP_INFO(get_logger(),
                    "Image saved: %s", path.c_str());

                current_task_index_++;
                run_next_task();
            }
        );
    }

    else {
        RCLCPP_WARN(get_logger(),
            "Unknown task type: %s", type.c_str());
        current_task_index_++;
        run_next_task();
    }
}

/* ============================= */
/*              main             */
/* ============================= */

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

    json tasks = request_from_llm(
        "去厨房看看桌上有没有红杯子");

    orchestrator->run_task_sequence(tasks);

    rclcpp::spin(orchestrator);
    rclcpp::shutdown();
    return 0;
}