#pragma once
#include <string>
#include <functional>
#include "geometry_msgs/msg/pose_stamped.hpp"

struct TaskResult { bool success; std::string reason; };

class RobotAdapter {
public:
    virtual ~RobotAdapter() = default;

    virtual void navigate_to(const geometry_msgs::msg::PoseStamped &pose,
                             std::function<void(TaskResult)> cb) = 0;

    virtual void observe(std::function<void(TaskResult, std::string image_path)> cb) = 0;

    virtual void follow_target(int target_id, std::function<void(TaskResult)> cb) = 0;

    virtual void stop(std::function<void(TaskResult)> cb) = 0;
};
