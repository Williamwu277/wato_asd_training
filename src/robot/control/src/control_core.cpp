#include "control_core.hpp"

namespace robot {

    ControlCore::ControlCore(const rclcpp::Logger& logger) :
    logger_(logger),
    std::make_shared<geometry_msgs::msg::Twist>() {}

    void ControlCore::createControlCore(double lookahead_distance, double goal_tolerance, double linear_speed) {
        lookahead_distance_ = lookahead_distance;
        goal_tolerance_ = goal_tolerance;
        linear_speed_ = linear_speed;
    }

    void ControlCore::updateControls() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }

        // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            return;  // No valid lookahead point found
        }

        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);
    }
}
