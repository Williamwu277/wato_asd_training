#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) :
resolution_ = 0.1;
width_ = 100;
height_ = 100;
origin_.position.x = -5.0;
origin_.position.y = -5.0;
origin_.orientation.w = 1.0;
inflation_radius_ = 1.0;
{

  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(
                &CostmapNode::laserScanCallback, this,
                std::placeholders::_1));
  pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  costmap_.createCostmap(resolution_, width_, height_, origin_, inflation_radius_);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserScanCallback(const sensor_msg::LaserScan::SharedPtr msg) {
    costmap_.updateCostmap(msg);
    nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
    costmap_msg.header = msg->header;
    pub_->publish(costmap_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}