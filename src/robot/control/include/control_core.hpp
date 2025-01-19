#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void createControlCore(double lookahead_distance, double goal_tolerance, double linear_speed);
    void updateControls();

    void setRobotOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {robot_odom_ = msg;}
    void setCurrentPath(const nav_msgs::msg::Path::SharedPtr msg) {current_path_ = msg;}
    geometry_msgs::msg::Twist::SharedPtr getControlData() const {return control_data_;}
  
  private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    rclcpp::Logger logger_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    geometry_msgs::msg::Twist::SharedPtr control_data_;

    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 
