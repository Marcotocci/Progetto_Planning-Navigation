#include "progetto_planning/take_screenshot_action.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <future>

using namespace std::chrono_literals;


TakeScreenshot::TakeScreenshot(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}


BT::NodeStatus TakeScreenshot::tick()
{

  if (photo_taken_) {
    return BT::NodeStatus::SUCCESS; 
  }

  
  auto photo_node = rclcpp::Node::make_shared("photo_snapper_node");
  
  
  std::promise<sensor_msgs::msg::Image::SharedPtr> image_promise;
  auto image_future = image_promise.get_future();

  
  auto sub = photo_node->create_subscription<sensor_msgs::msg::Image>(
    "/color/image_raw", 10,
    [&image_promise](const sensor_msgs::msg::Image::SharedPtr msg) {
      
      image_promise.set_value(msg);
    }
  );

  RCLCPP_INFO(photo_node->get_logger(), "[Screenshot] In attesa del frame video...");

  
  rclcpp::spin_until_future_complete(photo_node, image_future, 3s);

  
  if (image_future.wait_for(0s) == std::future_status::ready) {
    auto img_msg = image_future.get();
    try {
      //Convertiamo il messaggio ROS in formato BGR per OpenCV
      cv::Mat image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      
      
      
      const char* home_dir = std::getenv("HOME");
      std::string path;
      if (home_dir != nullptr) {
          
          path = std::string(home_dir) + "/ros2_ws/src/pkg/progetto_planning/alien_target_found.png";
      } else {
          path = "alien_target_found.png"; 
      }
      
      cv::imwrite(path, image);
      RCLCPP_INFO(photo_node->get_logger(), "\033[1;32m[Screenshot] CLICK! Foto salvata in: %s\033[0m", path.c_str());
      
      photo_taken_ = true;     
      
      return BT::NodeStatus::SUCCESS; 
      
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(photo_node->get_logger(), "[Screenshot] Errore cv_bridge: %s", e.what());
      return BT::NodeStatus::FAILURE; 
    }
  }

  
  RCLCPP_WARN(photo_node->get_logger(), "[Screenshot] Nessun dato dalla telecamera (Timeout).");
  return BT::NodeStatus::FAILURE; 
}


BT::PortsList TakeScreenshot::providedPorts()
{
  return {};
}