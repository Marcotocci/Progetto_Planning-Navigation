#include "progetto_planning/illumination_layer.hpp"

using namespace illumination_layer_namespace;

IlluminationLayer::IlluminationLayer() {}

void IlluminationLayer::onInitialize()
{
  auto node = node_.lock();

  node->declare_parameter("enabled", rclcpp::ParameterValue(true));
  node->declare_parameter("light_cost", rclcpp::ParameterValue(20));
  node->declare_parameter("shadow_cost", rclcpp::ParameterValue(150));
  node->declare_parameter("topic_name", rclcpp::ParameterValue(std::string("/illumination_data")));

  node->get_parameter("enabled", enabled_);
  node->get_parameter("light_cost", light_cost_);
  node->get_parameter("shadow_cost", shadow_cost_);
  node->get_parameter("topic_name", topic_name_);

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(10),
    std::bind(&IlluminationLayer::illuminationCallback, this, std::placeholders::_1));

  current_ = true;
  matchSize();
}

void IlluminationLayer::illuminationCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_grid_ = msg;
}

void IlluminationLayer::updateBounds(double, double, double,
                                     double* min_x, double* min_y,
                                     double* max_x, double* max_y)
{
  if (!enabled_ || !latest_grid_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  
  const auto& info = latest_grid_->info;

  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;
  double width = info.width * info.resolution;
  double height = info.height * info.resolution;

  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, origin_x + width);
  *max_y = std::max(*max_y, origin_y + height);
}

void IlluminationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                    int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_ || !latest_grid_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto& grid = latest_grid_;

  for (unsigned int i = 0; i < grid->info.width; ++i)
  {
    for (unsigned int j = 0; j < grid->info.height; ++j)
    {
      int val = grid->data[j * grid->info.width + i];

      // Ignoriamo i valori sconosciuti
      if (val == -1) continue;

      // Calcoliamo le coordinate mondo (x, y) di questa singola cella d'ombra
      double wx = grid->info.origin.position.x + (i + 0.5) * grid->info.resolution;
      double wy = grid->info.origin.position.y + (j + 0.5) * grid->info.resolution;

      // Verifichiamo se questa specifica cella cade dentro la master_grid
      unsigned int mx, my;
      if (!master_grid.worldToMap(wx, wy, mx, my)) {
        continue; // Se la cella è fuori, la ignoriamo
      }

      
      unsigned char old_cost = master_grid.getCost(mx, my);

      
      if (old_cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
          old_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
          old_cost == nav2_costmap_2d::NO_INFORMATION) {
          continue; 
      }

      
      unsigned char new_cost = old_cost;

      if (val == 0) {
          // Luce
          new_cost = std::max(old_cost, light_cost_);
      } else if (val > 0) {
          // Ombra
          new_cost = std::max(old_cost, shadow_cost_);
          
          // Sicurezza anti-blocco
          if (new_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
              new_cost = nav2_costmap_2d::LETHAL_OBSTACLE - 1;
          }
      }

      master_grid.setCost(mx, my, new_cost);
    }
  }
}

void IlluminationLayer::reset()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "IlluminationLayer reset");
}

PLUGINLIB_EXPORT_CLASS(illumination_layer_namespace::IlluminationLayer, nav2_costmap_2d::Layer)