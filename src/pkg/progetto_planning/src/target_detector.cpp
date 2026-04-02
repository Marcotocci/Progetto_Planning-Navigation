#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class TargetDetector : public rclcpp::Node {
public:
    TargetDetector() : Node("target_detector_node") {
        
        pub_ = this->create_publisher<std_msgs::msg::Bool>("/target_found", 10);
        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/color/detector_mask", 10);
        
       
        track_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_tracking", 10);
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 10,
            std::bind(&TargetDetector::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[1;36m[Detector] Avviato con Visual Servoing attivo.\033[0m");
        target_found_ = false;
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (target_found_) return; 

        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat hsv, mask;

            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::Scalar lower_bound(35, 100, 100); 
            cv::Scalar upper_bound(85, 255, 255);
            cv::inRange(hsv, lower_bound, upper_bound, mask);

            auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg();
            mask_pub_->publish(*mask_msg);
            
            int pixel_count = cv::countNonZero(mask);

            
            if (pixel_count > 500) {
                
                // Calcoliamo il Baricentro 
                cv::Moments M = cv::moments(mask);
                if (M.m00 > 0) {
                    int cx = int(M.m10 / M.m00); // Coordinata X del centro del target
                    int width = frame.cols;      // Larghezza totale dell'immagine

                    // Calcolo dell'angolo di errore (supponendo un FOV della telecamera di circa 60 gradi = 1.047 rad)
                    double fov = 1.047;
                    double yaw_error = -1.0 * (cx - width / 2.0) * (fov / width);

                    geometry_msgs::msg::Point track_msg;
                    track_msg.x = yaw_error; 
                    track_msg.y = pixel_count; 

                    
                    if (pixel_count > 10000) {
                        RCLCPP_INFO(this->get_logger(), "\033[1;32m[Detector] TARGET RAGGIUNTO! (%d px). MISSIONE COMPIUTA!\033[0m", pixel_count);
                        track_msg.z = 2.0; 
                        
                        
                        std_msgs::msg::Bool target_msg;
                        target_msg.data = true;
                        pub_->publish(target_msg); 
                        target_found_ = true; 
                    } 
                    
                    else {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "\033[1;33m[Detector] INSEGUIMENTO: Target a %.2f rad (Area: %d px)\033[0m", yaw_error, pixel_count);
                        track_msg.z = 1.0;
                    }

                    track_pub_->publish(track_msg);
                }
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[Detector] Errore cv_bridge: %s", e.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr track_pub_; 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    bool target_found_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDetector>());
    rclcpp::shutdown();
    return 0;
}