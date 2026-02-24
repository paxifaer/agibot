#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <fstream>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class ImageLatencyMonitor : public rclcpp::Node
{
public:
    ImageLatencyMonitor()
    : Node("image_latency_monitor",
           rclcpp::NodeOptions().append_parameter_override("use_sim_time", true))
    {
        file_.open("image_qos_comparison.csv");
        file_ << "qos,avg_ms,min_ms,max_ms,std_ms\n";

        start_test(false);  // 先 BestEffort

        timer_ = create_wall_timer(
            30s,
            std::bind(&ImageLatencyMonitor::switch_phase, this));
    }

private:

    void start_test(bool reliable)
    {
        latencies_.clear();
        current_reliable_ = reliable;

        rclcpp::QoS qos(10);
        if (reliable)
            qos.reliable();
        else
            qos.best_effort();

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            qos,
            std::bind(&ImageLatencyMonitor::cb, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Started %s test",
            reliable ? "Reliable" : "BestEffort");
    }

    void switch_phase()
    {
        sub_.reset();
        compute_and_save();

        if (!current_reliable_) {
            // 切换到 Reliable
            start_test(true);
        } else {
            // 测试完成
            RCLCPP_INFO(get_logger(), "QoS comparison finished.");
            file_.close();
            rclcpp::shutdown();
        }
    }

    void cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto now_time = now();
        auto sent_time = rclcpp::Time(msg->header.stamp);

        double latency =
            (now_time.seconds() - sent_time.seconds()) * 1000.0;

        if (latency > 0 && latency < 1000)
            latencies_.push_back(latency);
    }

    void compute_and_save()
    {
        if (latencies_.empty()) {
            RCLCPP_WARN(get_logger(), "No samples collected.");
            return;
        }

        double sum = 0;
        double min_v = latencies_[0];
        double max_v = latencies_[0];

        for (auto v : latencies_) {
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }

        double avg = sum / latencies_.size();

        double variance = 0;
        for (auto v : latencies_) {
            variance += (v - avg) * (v - avg);
        }
        variance /= latencies_.size();
        double stddev = std::sqrt(variance);

        file_ << (current_reliable_ ? "reliable" : "best_effort") << ","
              << avg << ","
              << min_v << ","
              << max_v << ","
              << stddev << "\n";

        RCLCPP_INFO(get_logger(),
            "[%s] avg=%.3f ms min=%.3f max=%.3f std=%.3f samples=%zu",
            current_reliable_ ? "reliable" : "best_effort",
            avg, min_v, max_v, stddev, latencies_.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::ofstream file_;
    std::vector<double> latencies_;
    bool current_reliable_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageLatencyMonitor>());
    return 0;
}