#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void createCostmap(double, resolution, int width_, int height_,
                       geometry_msgs::msg::Pose origin_, double inflation_radius_);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserScan);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
};

}  

#endif  