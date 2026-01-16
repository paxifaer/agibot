#include "task_orchestrator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <curl/curl.h>



size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// 从 LLM parser 获取 JSON
json request_from_llm(const std::string &text) {
    CURL* curl = curl_easy_init();
    std::string readBuffer;
    json result;

    if(curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        std::string postData = "{\"text\": \"" + text + "\"}";

        curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8000/parse");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_PROXY, "");

        CURLcode res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        } else {
            try {
                result = json::parse(readBuffer);
            } catch(...) {
                std::cerr << "Failed to parse JSON from LLM" << std::endl;
            }
        }
        curl_easy_cleanup(curl);
    }

    return result;
}

TaskOrchestrator::TaskOrchestrator(std::shared_ptr<RobotAdapter> adapter)
: rclcpp::Node("task_orchestrator", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)),
  adapter_(adapter)
{
}

#include <future> // 用于 promise/future

void TaskOrchestrator::run_task_sequence(const json &task_sequence) {
    for(const auto &task : task_sequence["tasks"]) {
        std::string type = task["type"];
        std::promise<TaskResult> prom;
        std::future<TaskResult> fut = prom.get_future();

        if(type == "navigate") {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = task["pose"]["x"];
            pose.pose.position.y = task["pose"]["y"];
            pose.pose.orientation.w = 1.0; // 简化

            adapter_->navigate_to(pose, [&prom](TaskResult res){
                prom.set_value(res); // 回调完成时设置 promise
            });
        } else if(type == "observe") {
            adapter_->observe([&prom](TaskResult res, std::string image_path){
                if(res.success)
                    RCLCPP_INFO(rclcpp::get_logger("TaskOrchestrator"), "Observed: %s", image_path.c_str());
                else
                    RCLCPP_WARN(rclcpp::get_logger("TaskOrchestrator"), "Observe failed");
                prom.set_value(res); // 回调完成时设置 promise
            });
        }

        // 阻塞等待任务完成
        TaskResult result = fut.get();  // 这里会真正等待异步完成
        if(!result.success) {
            RCLCPP_WARN(rclcpp::get_logger("TaskOrchestrator"), "Task failed, stopping sequence.");
            break; // 可选：失败就停止后续任务
        }
    }
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

  //   auto demo_json = R"({
  //     "tasks": [
  //       { "type": "navigate", "x": 0.1, "y": 0.0, "yaw": 0.0  }
  //     ]
  //   })";

  // orchestrator->execute_task_json(demo_json);

  json tasks = request_from_llm("去厨房看看桌上有没有红杯子");
  orchestrator->run_task_sequence(tasks);

  rclcpp::spin(orchestrator);
  rclcpp::shutdown();
  return 0;
}