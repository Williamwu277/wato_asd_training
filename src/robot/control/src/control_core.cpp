#include "control_core.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace robot {

    ControlCore::ControlCore(const rclcpp::Logger& logger) :
    logger_(logger),
    std::make_shared<geometry_msgs::msg::Twist>() {}

    void ControlCore::createControlCore(double lookahead_distance, double goal_tolerance,
                                        double linear_speed, double max_steer_angle) {
        lookahead_distance_ = lookahead_distance;
        goal_tolerance_ = goal_tolerance;
        linear_speed_ = linear_speed;
        max_steer_angle_ = max_steer_angle;
    }

    void ControlCore::updateControls() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }

        // Find the lookahead point
        geometry_msgs::msg::PoseStamped lookahead_point = findLookaheadPoint();
        if (!lookahead_point.has_value()) {
            return;
        }

        // Compute velocity command
        auto cmd_vel = computeVelocity(lookahead_point);
    }

    std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
        geometry_msgs::msg::PoseStamped lookahead_point;
        double robot_x = robot_odom_->pose.pose.position.x;
        double robot_y = robot_odom_->pose.pose.position.y;
        double robot_theta = extractYaw(robot_odom_->pose.pose.orientation);

        int lookahead_index;
        double min_distance = std::numeric_limits<double>::max();
        for (int i = 0; i < current_path_->poses.size(); ++i) {
            double dx = current_path_->poses[i].pose.position.x - robot_x;
            double dy = current_path_->poses[i].pose.position.y - robot_y;
            double distance = std::hypot(dx, dy);
            if (i == current_path_->poses.size() && distance < lookahead_distance_) {
                lookahead_point.pose = poses[i];
                break;
            }

            if (distance < lookahead_distance_) continue;

            double angle_to_point = std::atan2(dy, dx);

            double angle_diff = angle_to_point - robot_theta;

            if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            if (std::abs(angle_diff) < M_PI / 2) {
                if (distance < min_distance) {
                    min_distance = distance;
                    lookahead_index = i;
                    break;
                }
            }
        }
        lookahead_point.pose = poses[lookahead_index];

        return lookahead_point;
    }

    void ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        double target_x = target.pose.position.x;
        double target_y = target.pose.position.y;

        double robot_x = robot_odom_->pose.pose.position.x;
        double robot_y = robot_odom_->pose.pose.position.y;
        double robot_theta = extractYaw(robot_odom_->pose.pose.orientation);

        double dx = target_x - robot_x;
        double dy = target_y - robot_y;

        //arrive within goal_tolerance
        if (std::hypot(dx, dy) < goal_tolerance_) {
            control_data_.linear.x = 0;
            control_data_.angular.z = 0;
            return;
        }

        double angle_to_target = std::atan(dy, dx);
        double steer_angle = angle_to_target - robot_theta;

        if (steer_angle > M_PI) {
            steer_angle -= 2 * M_PI;
        } else if (steer_angle < -M_PI) {
            steer_angle += 2 * M_PI;
        }

        if (steer_angle > max_steering_angle_) {
            control_data_.linear.x = 0;
        } else {
            control_data_.linear.x = linear_speed_;
        }

        control_data_.angular.z = steer_angle;
    }

    double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
}
