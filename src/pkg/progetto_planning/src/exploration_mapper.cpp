#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class ExplorationMapper : public rclcpp::Node {
public:
    ExplorationMapper() : Node("exploration_mapper"), map_initialized_(false) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10,
            std::bind(&ExplorationMapper::mapCallback, this, std::placeholders::_1));

        
        rclcpp::QoS map_pub_qos(10);
        map_pub_qos.transient_local();
        map_pub_qos.reliable();

        explored_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/explored_area", map_pub_qos);
        
        timer_ = this->create_wall_timer(500ms, std::bind(&ExplorationMapper::updateExploredArea, this));
        
        RCLCPP_INFO(this->get_logger(), "Exploration Mapper avviato. In attesa della mappa...");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_initialized_) {
           
            map_frame_ = msg->header.frame_id;
            
            explored_map_.header = msg->header;
            explored_map_.header.frame_id = "rover/map";
            explored_map_.info = msg->info;
            explored_map_.data.assign(msg->info.width * msg->info.height, 0); 
            map_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Mappa inizializzata (%d x %d). Frame: %s", 
                        msg->info.width, msg->info.height, map_frame_.c_str());
        }
    }

    void updateExploredArea() {
        if (!map_initialized_) return;

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(map_frame_, "rover/base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Attesa TF (%s -> rover/base_footprint): %s", map_frame_.c_str(), ex.what());
            return; 
        }

        double rx = t.transform.translation.x;
        double ry = t.transform.translation.y;
        double res = explored_map_.info.resolution;
        
        double explore_radius_m = 2.5; 
        int radius_cells = std::ceil(explore_radius_m / res);

        int center_x = (rx - explored_map_.info.origin.position.x) / res;
        int center_y = (ry - explored_map_.info.origin.position.y) / res;

        bool updated = false;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (dx*dx + dy*dy <= radius_cells*radius_cells) {
                    int nx = center_x + dx;
                    int ny = center_y + dy;

                    if (nx >= 0 && nx < (int)explored_map_.info.width && ny >= 0 && ny < (int)explored_map_.info.height) {
                        int index = ny * explored_map_.info.width + nx;
                        explored_map_.data[index] = 100; 
                        updated = true;
                    }
                }
            }
        }

        if (updated) {
            explored_map_.header.stamp = this->now();
            explored_pub_->publish(explored_map_);
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr explored_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid explored_map_;
    std::string map_frame_;
    bool map_initialized_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorationMapper>());
    rclcpp::shutdown();
    return 0;
}