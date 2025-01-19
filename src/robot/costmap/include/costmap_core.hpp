#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <utility>
#include <unordered_map>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    bool in_grid(int grid_x, int grid_y);

    //creating costmap
    void createCostmap(double resolution, int width, int height,
                       geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost);

    //updating costmap
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const {return costmap_data_;}

  private:
    void calculateInflationMask();
    void inflateObstacle(int grid_x, int grid_y);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    double inflation_radius_;
    int max_cost_;
    std::unordered_map<std::pair<int, int>, int> inflation_mask_;
};

}  

#endif  