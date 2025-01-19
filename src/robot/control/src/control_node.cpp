#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) :
lookahead_distance_ = 1.0,  // Lookahead distance
goal_tolerance_ = 0.1,     // Distance to consider the goal reached
linear_speed_ = 0.5 {
    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { control_.setCurrentPath(msg); });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { control_.setRobotOdom(msg); });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });

    control_.createControlCore(lookahead_distance_, goal_tolerance_, linear_speed_);
}

void ControlNode::controlLoop() {
    control_.updateControls();
    geometry_msgs::msg::Twist cmd_vel = *control_.getControlData();
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
