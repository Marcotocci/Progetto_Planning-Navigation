#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <random>
#include <atomic>
#include <vector>
#include <limits>
#include <cmath>
#include <chrono>
#include <string>
#include <mutex>
#include <algorithm>

class ExploreUntilTargetFound : public BT::StatefulActionNode {
public:
  using PlanAction = nav2_msgs::action::ComputePathToPose;
  using PlanHandle = rclcpp_action::ClientGoalHandle<PlanAction>;
  using NavAction  = nav2_msgs::action::NavigateToPose;
  using NavHandle  = rclcpp_action::ClientGoalHandle<NavAction>;

  struct Candidate {
    double x{0.0}, y{0.0}, yaw{0.0}, dist{0.0};
  };

  ExploreUntilTargetFound(const std::string& name,
                          const BT::NodeConfiguration& cfg,
                          const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, cfg), node_(node)
  {
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_);

    sub_target_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/target_found", 10,
      [this](const std_msgs::msg::Bool::SharedPtr m){ target_found_.store(m->data); });

    
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    
    sub_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::lock_guard<std::recursive_mutex> lk(map_mtx_);
        map_ = msg;
        map_frame_ = map_->header.frame_id;
        res_ = map_->info.resolution;
        width_ = map_->info.width;
        height_ = map_->info.height;
        origin_x_ = map_->info.origin.position.x;
        origin_y_ = map_->info.origin.position.y;
      });

    rng_.seed(std::random_device{}());
  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("frame_id",         std::string("map"),            "global frame id"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_footprint"), "robot base frame"),
      BT::InputPort<std::string>("plan_server_name", std::string("compute_path_to_pose"), "ComputePathToPose action name"),
      BT::InputPort<std::string>("nav_server_name",  std::string("navigate_to_pose"),     "NavigateToPose action name"),
      BT::InputPort<std::string>("server_name",      std::string("navigate_to_pose"),     "Alias per nav_server"),
      BT::InputPort<int>("n_rays",            24,   "numero di raggi (angoli)"),
      BT::InputPort<double>("max_scan_range_m", 8.0,"raggio max scansione su mappa"),
      BT::InputPort<double>("backoff_m",      0.4,  "arretramento dal bordo ignoto"),
      BT::InputPort<int>("clearance_cells_min", 2,  "clearance minima in celle"),
      BT::InputPort<int>("min_path_poses",   5,     "scarta piani troppo corti"),
      BT::InputPort<int>("goal_timeout_ms",  90000, "timeout per-goal"),
      BT::InputPort<double>("min_progress_m",     0.20, "Δd minimo per considerare progresso"),
      BT::InputPort<int>("progress_timeout_ms",   5000, "tempo max senza progresso"),
      BT::InputPort<int>("max_failures",     8,     "limite fallimenti consecutivi"),
      BT::InputPort<double>("map_margin_m",  0.4,   "margine interno dai bordi mappa"),
      BT::InputPort<int>("free_threshold",   20,    "valore max (0..100) per considerare free")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { PLANNING, NAVIGATING };

  bool getRobotPose(double& x, double& y, double& yaw);
  geometry_msgs::msg::Quaternion yawToQuat(double yaw){
    geometry_msgs::msg::Quaternion q;
    q.x=0; q.y=0; q.z=std::sin(yaw*0.5); q.w=std::cos(yaw*0.5);
    return q;
  }

  bool buildRayFrontierCandidates(double rx_world, double ry_world);
  bool rayToFrontier(double rx_m, double ry_m, double ang_m,
                     double& goal_x_world, double& goal_y_world, double& yaw_world);
  bool clearanceOKCells(int mx, int my, int min_cells) const;

  bool sendPlanRequest();
  bool sendNavGoal();
  void cancelNav(bool wait=false);

  bool haveMap() const { std::lock_guard<std::recursive_mutex> lk(map_mtx_); return (bool)map_; }
  bool toMapFrame(double x_in, double y_in, double& x_map, double& y_map);
  bool insideMapBounds(double x, double y, double margin_m) const;
  bool worldToMap(double x, double y, int& mx, int& my) const;
  int  cellAt(int mx, int my) const;
  bool isFreeCell(int mx, int my) const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_target_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;

  rclcpp_action::Client<PlanAction>::SharedPtr plan_client_;
  rclcpp_action::Client<NavAction>::SharedPtr  nav_client_;

  std::atomic<bool> plan_request_in_flight_{false};
  typename PlanHandle::SharedPtr plan_handle_;
  std::shared_future<typename PlanHandle::WrappedResult> plan_result_future_;
  bool plan_result_requested_{false};

  std::atomic<bool> nav_request_in_flight_{false};
  typename NavHandle::SharedPtr  nav_handle_;
  std::shared_future<typename NavHandle::WrappedResult> nav_result_future_;
  bool nav_result_requested_{false};
  bool nav_active_{false};

  std::vector<Candidate> candidates_;
  size_t cand_idx_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr explored_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr explored_map_;
  void exploredCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr track_sub_;
  bool tracking_active_ = false;
  double target_yaw_error_ = 0.0;
  double target_dist_estimate_ = 0.0;
  rclcpp::Time last_track_time_;
  void trackCallback(const geometry_msgs::msg::Point::SharedPtr msg);

  std::string frame_id_{"map"};
  std::string base_frame_{"base_footprint"};
  std::string plan_server_{"compute_path_to_pose"};
  std::string nav_server_{"navigate_to_pose"};


  int n_rays_{24};
  double max_scan_range_m_{8.0};
  double backoff_m_{0.4};
  int clearance_cells_min_{2};
  int min_path_poses_{5};
  int max_failures_{8};
  rclcpp::Duration goal_timeout_{0,0};
  double min_progress_m_{0.20};
  rclcpp::Duration progress_timeout_{0,0};
  double last_feedback_dist_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_progress_time_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::string map_frame_{};
  mutable std::recursive_mutex map_mtx_;
  double res_{0.0};
  uint32_t width_{0}, height_{0};
  double origin_x_{0.0}, origin_y_{0.0};
  double map_margin_m_{0.4};
  int free_threshold_{20};

  rclcpp::Time nav_start_time_;
  int failures_{0};
  std::atomic<bool> target_found_{false};
  Phase phase_{Phase::PLANNING};
  double last_goal_x_{0.0}, last_goal_y_{0.0}, last_goal_yaw_{0.0};
  std::mt19937 rng_;
};