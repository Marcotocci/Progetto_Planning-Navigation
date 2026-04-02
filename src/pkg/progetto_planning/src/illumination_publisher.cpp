#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>

#include <mutex>
#include <string>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class IlluminationPublisher : public rclcpp::Node {
public:
  IlluminationPublisher() : Node("illumination_publisher") {
    declare_parameter<std::string>("illumination_yaml_path", "shadow_map.yaml");
    declare_parameter<std::string>("map_frame", "rover/map");
    declare_parameter<std::string>("robot_frame", "rover/base_footprint");
    declare_parameter<int>("window_size", 100);

    get_parameter("illumination_yaml_path", illum_yaml_path_);
    get_parameter("map_frame", map_frame_);
    get_parameter("robot_frame", robot_frame_);
    get_parameter("window_size", window_size_);

    RCLCPP_INFO(get_logger(),
      "Avvio Illumination Publisher. Parametri: illum_yaml='%s', window_size=%d",
      illum_yaml_path_.c_str(), window_size_);

    loadIlluminationFromFile();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_   = create_publisher<nav_msgs::msg::OccupancyGrid>("/illumination_data", 1);
    timer_ = create_wall_timer(500ms, std::bind(&IlluminationPublisher::tick, this));
  }

private:
  void loadIlluminationFromFile() {
    YAML::Node cfg;
    try {
      cfg = YAML::LoadFile(illum_yaml_path_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Impossibile leggere il file YAML: %s", e.what());
      throw;
    }

    std::string img_rel = cfg["image"].as<std::string>();
    double res = cfg["resolution"].as<double>();
    double ox  = cfg["origin"][0].as<double>();
    double oy  = cfg["origin"][1].as<double>();

    std::filesystem::path y(illum_yaml_path_), p(img_rel);
    if (!p.is_absolute()) p = y.parent_path() / p;
    auto img_path = p.lexically_normal().string();

    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      RCLCPP_FATAL(get_logger(), "Immagine GIMP non trovata: %s", img_path.c_str());
      throw std::runtime_error("Immagine non caricata");
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame_; 
    grid.info.resolution = res;
    grid.info.width  = static_cast<uint32_t>(img.cols);
    grid.info.height = static_cast<uint32_t>(img.rows);
    grid.info.origin.position.x = ox;
    grid.info.origin.position.y = oy;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<size_t>(grid.info.width * grid.info.height));

    for (uint32_t yrow = 0; yrow < grid.info.height; ++yrow) {
      int yy = static_cast<int>(grid.info.height - 1 - yrow);
      const uint8_t* row = img.ptr<uint8_t>(yy);
      for (uint32_t xcol = 0; xcol < grid.info.width; ++xcol) {
        uint8_t v = row[xcol];
        int8_t out;
        
        if (v == 205) {
            out = -1; 
        } else {
            // Formula sfumature ombre proporzionali
            out = static_cast<int8_t>((255 - v) * 100.0 / 255.0);
        }
        grid.data[static_cast<size_t>(yrow) * grid.info.width + xcol] = out;
      }
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      illum_grid_ = std::move(grid);
      have_illum_ = true;
    }
    RCLCPP_INFO(get_logger(), "Mappa ombre di GIMP caricata in memoria con successo!");
  }

  void tick() {
    if (!have_illum_) return;

    nav_msgs::msg::OccupancyGrid illum;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      illum = illum_grid_;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      return; // In attesa che il robot pubblichi la sua posizione
    }

    const double res = illum.info.resolution;
    const double ox  = illum.info.origin.position.x;
    const double oy  = illum.info.origin.position.y;
    const uint32_t W = illum.info.width;
    const uint32_t H = illum.info.height;

    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;

    const int cx = static_cast<int>(std::floor((rx - ox) / res));
    const int cy = static_cast<int>(std::floor((ry - oy) / res));
    
    // Se il robot esce dai confini della mappa, non pubblichiamo nulla
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(W) || cy >= static_cast<int>(H)) return;

    const int half = window_size_ / 2;
    const int sx = std::max(0, cx - half);
    const int sy = std::max(0, cy - half);
    const int ex = std::min<int>(W, cx + half);
    const int ey = std::min<int>(H, cy + half);

    const int subW = std::max(0, ex - sx);
    const int subH = std::max(0, ey - sy);
    if (subW <= 0 || subH <= 0) return;

    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    out.info.resolution = res;
    out.info.width  = static_cast<uint32_t>(subW);
    out.info.height = static_cast<uint32_t>(subH);
    out.info.origin.position.x = ox + sx * res;
    out.info.origin.position.y = oy + sy * res;
    out.info.origin.orientation.w = 1.0;
    out.data.resize(static_cast<size_t>(subW * subH));

    for (int y = 0; y < subH; ++y) {
      const int yy = sy + y;
      const size_t src_row = static_cast<size_t>(yy) * W;
      const size_t dst_row = static_cast<size_t>(y)  * subW;
      for (int x = 0; x < subW; ++x) {
        out.data[dst_row + x] = illum.data[src_row + static_cast<size_t>(sx + x)];
      }
    }

    pub_->publish(out);
  }

  std::string illum_yaml_path_, map_frame_, robot_frame_;
  int window_size_{100};

  nav_msgs::msg::OccupancyGrid illum_grid_;
  bool have_illum_{false};

  std::mutex mutex_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IlluminationPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}