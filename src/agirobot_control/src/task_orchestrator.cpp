#include "task_orchestrator.hpp"
#include "agirobot_control/turtlebot3_adapter.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <curl/curl.h>
#include <future>
#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>

using json = nlohmann::json;

static size_t WriteCallback(void* contents, size_t size,
                            size_t nmemb, void* userp)
{
    size_t total = size * nmemb;
    ((std::string*)userp)->append((char*)contents, total);
    return total;
}

static json request_from_llm(const std::string &text)
{
    CURL* curl = curl_easy_init();
    std::string buffer;
    json result;

    if (!curl)
        return result;

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

    curl_easy_perform(curl);

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    try {
        result = json::parse(buffer);
    }
    catch (...) {
        std::cerr << "JSON parse error\n";
    }

    return result;
}

/* ===================================================== */
/*                 TaskOrchestrator 实现                 */
/* ===================================================== */

TaskOrchestrator::TaskOrchestrator()
: Node("task_orchestrator",
       rclcpp::NodeOptions().append_parameter_override("use_sim_time", true))
{
    perf_file_.open("e2e_latency.csv");
    perf_file_ << "task_type,start_ns,end_ns,latency_ms\n";
}

TaskOrchestrator::~TaskOrchestrator()
{
    if (perf_file_.is_open()) {
        perf_file_.flush();
        perf_file_.close();
    }
}

void TaskOrchestrator::set_adapter(std::shared_ptr<RobotAdapter> adapter)
{
    adapter_ = adapter;
}

void TaskOrchestrator::run_task_sequence(const json &tasks)
{
    if (!tasks.contains("tasks") || !tasks["tasks"].is_array()) {
        RCLCPP_ERROR(get_logger(), "Invalid task JSON format");
        return;
    }

    RCLCPP_INFO(get_logger(), "Start task sequence");

    for (auto &task : tasks["tasks"]) {

        std::string type = task["type"];
        RCLCPP_INFO(get_logger(), "Execute: %s", type.c_str());

        if (type == "navigate") {

            auto pose_json = task["pose"];

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = now();
            pose.pose.position.x = pose_json.value("x", 0.0);
            pose.pose.position.y = pose_json.value("y", 0.0);
            pose.pose.orientation.w = 1.0;

            wait_nav(pose);
        }
        else if (type == "observe") {
            wait_observe();
        }
        else {
            RCLCPP_WARN(get_logger(),
                        "Unknown task type: %s", type.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "All tasks finished");

    perf_file_.flush();
}

/* ---------------- wait_nav ---------------- */

void TaskOrchestrator::wait_nav(
    const geometry_msgs::msg::PoseStamped &pose)
{
    auto start = std::chrono::steady_clock::now();

    std::promise<void> done;
    auto future = done.get_future();

    adapter_->navigate_to(pose,
        [&](TaskResult){
            done.set_value();
        });

    future.wait();

    auto end = std::chrono::steady_clock::now();
    auto latency =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    perf_file_ << "navigate,"
               << start.time_since_epoch().count() << ","
               << end.time_since_epoch().count() << ","
               << latency << "\n";
}

/* ---------------- wait_observe ---------------- */

void TaskOrchestrator::wait_observe()
{
    auto start = std::chrono::steady_clock::now();

    std::promise<void> done;
    auto future = done.get_future();

    adapter_->observe(
        [&](TaskResult, std::string path){
            RCLCPP_INFO(get_logger(),
                        "Image saved: %s", path.c_str());
            done.set_value();
        });

    future.wait();

    auto end = std::chrono::steady_clock::now();
    auto latency =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    perf_file_ << "observe,"
               << start.time_since_epoch().count() << ","
               << end.time_since_epoch().count() << ","
               << latency << "\n";
}

/* ===================================================== */
/*                        main                           */
/* ===================================================== */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto orchestrator =
        std::make_shared<TaskOrchestrator>();

    auto adapter =
        std::make_shared<TurtleBot3Adapter>(orchestrator);

    orchestrator->set_adapter(adapter);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(orchestrator);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    while (!adapter->is_action_server_ready()) {
        RCLCPP_INFO(orchestrator->get_logger(),
                    "Waiting for Nav2 action server...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    json tasks = request_from_llm(
        "去厨房看看桌上有没有红杯子");

    orchestrator->run_task_sequence(tasks);

    executor.cancel();
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}