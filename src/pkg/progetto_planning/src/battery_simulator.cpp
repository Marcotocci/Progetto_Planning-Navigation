#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>

using namespace std::chrono_literals;

class BatterySimulator : public rclcpp::Node {
public:
  BatterySimulator() : Node("battery_simulator"), charge_(100.0), is_in_light_(false), is_dead_(false) {
    declare_parameter("discharge_rate", 1.0);
    declare_parameter("charge_rate", 0.3); 
    get_parameter("discharge_rate", discharge_rate_);
    get_parameter("charge_rate", charge_rate_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    battery_pub_ = create_publisher<std_msgs::msg::Float32>("battery_status", 10);
    
    illumination_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/illumination_data", 10,
      std::bind(&BatterySimulator::illuminationCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(1s, std::bind(&BatterySimulator::updateBattery, this));
  }

private:
  void illuminationCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_msg_ = msg;
  }

  void updateBattery() {
    
    if (is_dead_) {
        std_msgs::msg::Float32 msg;
        msg.data = 0.0f;
        battery_pub_->publish(msg);
        return;
    }

    int current_light_val = -1;

    if (map_msg_) {
      try {
        auto t = tf_buffer_->lookupTransform("rover/map", "rover/base_footprint", tf2::TimePointZero);
        double rx = t.transform.translation.x;
        double ry = t.transform.translation.y;

        double origin_x = map_msg_->info.origin.position.x;
        double origin_y = map_msg_->info.origin.position.y;
        double res = map_msg_->info.resolution;
        
        int map_x = (rx - origin_x) / res;
        int map_y = (ry - origin_y) / res;

        if (map_x >= 0 && map_x < (int)map_msg_->info.width && map_y >= 0 && map_y < (int)map_msg_->info.height) {
          current_light_val = map_msg_->data[map_y * map_msg_->info.width + map_x];
          is_in_light_ = (current_light_val >= 0 && current_light_val <= 40);
        } else {
          is_in_light_ = false;
        }
      } catch (const tf2::TransformException & ex) {
        is_in_light_ = false; 
      }
    }

    if (is_in_light_) charge_ += charge_rate_;
    else charge_ -= discharge_rate_;

    charge_ = std::min(std::max(charge_, 0.0f), 100.0f);

    
    if (charge_ <= 0.0f) {
        is_dead_ = true;
        RCLCPP_FATAL(this->get_logger(), 
          "\n\n\t\033[1;41m [MISSION OVER] BATTERIA ALLO 0%%! IL ROVER È MORTO. \033[0m\n");
    }

    std_msgs::msg::Float32 msg;
    msg.data = charge_;
    battery_pub_->publish(msg);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr illumination_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;

  float charge_;
  float discharge_rate_;
  float charge_rate_;
  bool is_in_light_;
  bool is_dead_; 
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySimulator>());
  rclcpp::shutdown();
  return 0;
}