#ifndef ILLUMINATION_LAYER_HPP_
#define ILLUMINATION_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>

namespace illumination_layer_namespace
{

class IlluminationLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  IlluminationLayer();
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override { return false; }
  void reset() override;

private:
  void illuminationCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::mutex data_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;

  bool enabled_;
  unsigned char light_cost_;
  unsigned char shadow_cost_;
  std::string topic_name_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};

}  

#endif  
