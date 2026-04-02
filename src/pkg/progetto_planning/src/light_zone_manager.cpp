#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <limits>
#include <memory>

using namespace std::chrono_literals;

class LightZoneManager : public rclcpp::Node {
public:
    LightZoneManager() : Node("light_zone_manager") {
       
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/illumination_data", 10,
            std::bind(&LightZoneManager::mapCallback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/light_zone_goal", 10);
        
        timer_ = this->create_wall_timer(2s, std::bind(&LightZoneManager::findAndPublishLight, this));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_msg_ = msg;
    }

    void findAndPublishLight() {
        if (!map_msg_) return;

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("rover/map", "rover/base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; 
        }

        double rx = t.transform.translation.x;
        double ry = t.transform.translation.y;

        double origin_x = map_msg_->info.origin.position.x;
        double origin_y = map_msg_->info.origin.position.y;
        double res = map_msg_->info.resolution;
        int width = map_msg_->info.width;
        int height = map_msg_->info.height;

        double min_dist_sq = std::numeric_limits<double>::max();
        double best_x = rx, best_y = ry;
        bool found = false;


        int clearance_cells = 5; 

        // Scannerizziamo l'intera mappa
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Se troviamo un pixel di luce potenziale (0)
                if (map_msg_->data[y * width + x] == 0) {
                    
                    bool is_safe = true;
                    
                    // Controlliamo il "quadrato" attorno a questo pixel
                    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
                        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
                            int ny = y + dy;
                            int nx = x + dx;
                            
                            // Se il controllo esce dalla mappa, scartiamo il punto
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                // Se c'è un ostacolo o un'ombra (!= 0) nelle vicinanze, scartiamo il punto
                                if (map_msg_->data[ny * width + nx] != 0) {
                                    is_safe = false;
                                    break;
                                }
                            } else {
                                is_safe = false;
                                break;
                            }
                        }
                        if (!is_safe) break;
                    }

                    // Se il punto ha superato il test (è luce e ha spazio intorno)
                    if (is_safe) {
                        double px = origin_x + (x + 0.5) * res;
                        double py = origin_y + (y + 0.5) * res;
                        
                        double dx = px - rx;
                        double dy = py - ry;
                        double dist_sq = dx * dx + dy * dy;

                        // Salviamo il punto se è il più vicino trovato finora
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_x = px;
                            best_y = py;
                            found = true;
                        }
                    }
                }
            }
        }
        

        if (found) {
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp.sec = 0; 
            goal.header.stamp.nanosec = 0;
            goal.header.frame_id = "rover/map"; 
            goal.pose.position.x = best_x;
            goal.pose.position.y = best_y;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);
            
            /*RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "\033[1;36m[Radar Globale]\033[0m Trovata luce a X:%.2f Y:%.2f", best_x, best_y);*/
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightZoneManager>());
    rclcpp::shutdown();
    return 0;
}