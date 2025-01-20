#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>

namespace robot {

    CostmapCore::CostmapCore(const rclcpp::Logger& logger) :
    logger_(logger),
    costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

    void CostmapCore::createCostmap(double resolution, int width, int height,
                                    geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost) {
        costmap_data_->info.resolution = resolution;
        costmap_data_->info.width = width;
        costmap_data_->info.height = height;
        costmap_data_->info.origin = origin;
        costmap_data_->data.assign(width * height, 0);
        inflation_radius_ = inflation_radius;
        max_cost_ = max_cost;

        calculateInflationMask();
    }

    void CostmapCore::calculateInflationMask() {

        int cell_radius = static_cast<int>(inflation_radius_ / costmap_data_->info.resolution);

        inflation_mask_[{0, 0}] = 100; //obstacle;
        for (int i = -cell_radius; i <= cell_radius; ++i) {
            for (int j = -cell_radius; j <= cell_radius; ++j) {
                if (i == 0 && j == 0) {
                    continue;
                }
                double distance = std::hypot(i, j)/costmap_data_->info.resolution;
                if (distance <= inflation_radius_) {
                    int cost = static_cast<int>(max_cost_*(1-distance/inflation_radius_));
                    inflation_mask_[{i, j}] = cost;
                }
            }
        }
    }

    void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
        std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

        double angle = laser_scan->angle_min;
        for (int i = 0; i < static_cast<int>(laser_scan->ranges.size()); ++i, angle += laser_scan->angle_increment) {
            double range = laser_scan->ranges[i];
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            int grid_x = static_cast<int>((x + costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
            int grid_y = static_cast<int>((y + costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

            if (in_grid(grid_x, grid_y)) {
                inflateObstacle(grid_x, grid_y);
            }
        }
    }

    void CostmapCore::inflateObstacle(int grid_x, int grid_y) {
        for (auto it = inflation_mask_.begin(); it != inflation_mask_.end(); ++it) {
            std::pair coords = it->first;
            int cur_x = grid_x + coords.first;
            int cur_y = grid_y + coords.second;
            int cur_cost = it->second;
            if (in_grid(cur_x, cur_y)) {
                int index = cur_y * costmap_data_->info.width + cur_x;
                costmap_data_->data[index] = std::max(costmap_data_->data[index], (int8_t) cur_cost);
            }
        }
    }

    bool CostmapCore::in_grid(int grid_x, int grid_y) {
        return (grid_x >= 0 && grid_x < static_cast<int>(costmap_data_->info.width)) &&
        (grid_y >= 0 && grid_y < static_cast<int>(costmap_data_->info.height));
    }
}