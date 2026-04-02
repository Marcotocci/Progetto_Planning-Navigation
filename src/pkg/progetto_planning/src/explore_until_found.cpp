#include "progetto_planning/explore_until_found.hpp" 
#include <limits>

using namespace std::chrono_literals;

BT::NodeStatus ExploreUntilTargetFound::onStart()
{
  if(auto v = getInput<std::string>("frame_id"); v)         frame_id_   = v.value();
  if(auto v = getInput<std::string>("robot_base_frame"); v) base_frame_ = v.value();
  if(auto v = getInput<std::string>("plan_server_name"); v) plan_server_= v.value();
  if(auto v = getInput<std::string>("nav_server_name"); v)  nav_server_ = v.value();
  else if(auto v2 = getInput<std::string>("server_name"); v2) nav_server_ = v2.value(); 

  if(auto v = getInput<int>("n_rays"); v)               n_rays_ = v.value();
  if(auto v = getInput<double>("max_scan_range_m"); v)  max_scan_range_m_ = v.value();
  if(auto v = getInput<double>("backoff_m"); v)         backoff_m_ = v.value();
  if(auto v = getInput<int>("clearance_cells_min"); v)  clearance_cells_min_ = v.value();

  if(auto v = getInput<int>("min_path_poses"); v)       min_path_poses_ = v.value();
  if(auto v = getInput<int>("goal_timeout_ms"); v)      goal_timeout_ = rclcpp::Duration(std::chrono::milliseconds(v.value()));
  if(auto v = getInput<double>("min_progress_m"); v)    min_progress_m_ = v.value();
  if(auto v = getInput<int>("progress_timeout_ms"); v)  progress_timeout_ = rclcpp::Duration(std::chrono::milliseconds(v.value()));
  if(auto v = getInput<int>("max_failures"); v)         max_failures_ = v.value();

  if(auto v = getInput<double>("map_margin_m"); v)      map_margin_m_ = v.value();
  if(auto v = getInput<int>("free_threshold"); v)       free_threshold_ = v.value();

  last_feedback_dist_ = std::numeric_limits<double>::infinity();
  last_progress_time_ = node_->now();

  if(!plan_client_) plan_client_ = rclcpp_action::create_client<PlanAction>(node_, plan_server_);
  if(!nav_client_)  nav_client_  = rclcpp_action::create_client<NavAction>(node_,  nav_server_);

  // Inizializziamo il Subscriber per la mappa di esplorazione (Transient Local)
  if (!explored_sub_) {
    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    explored_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/explored_area", map_qos,
        std::bind(&ExploreUntilTargetFound::exploredCallback, this, std::placeholders::_1));
  }

  track_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
      "/target_tracking", 10,
      std::bind(&ExploreUntilTargetFound::trackCallback, this, std::placeholders::_1));

  if(!plan_client_->wait_for_action_server(2s) || !nav_client_->wait_for_action_server(2s)){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[Explore] action servers non pronti, attendo...");
    return BT::NodeStatus::RUNNING;
  }

  failures_ = 0;
  nav_active_ = false;
  nav_request_in_flight_.store(false);
  plan_request_in_flight_.store(false);
  nav_result_requested_  = false;
  plan_result_requested_ = false;
  nav_handle_.reset();
  plan_handle_.reset();
  candidates_.clear();
  cand_idx_ = 0;

  if(target_found_.load()){
    RCLCPP_INFO(node_->get_logger(), "[Explore] target già trovato");
    return BT::NodeStatus::SUCCESS;
  }

  if(!haveMap()){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[Explore] in attesa della /map...");
    return BT::NodeStatus::RUNNING;
  }

  double rx, ry, ryaw;
  if(getRobotPose(rx, ry, ryaw)){
    if(buildRayFrontierCandidates(rx, ry)){
      phase_ = Phase::PLANNING;
      (void)sendPlanRequest();
    } else {
      RCLCPP_INFO(node_->get_logger(), "[Explore] nessuna frontiera visibile al momento");
      return BT::NodeStatus::RUNNING;
    }
  } else {
    phase_ = Phase::PLANNING;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExploreUntilTargetFound::onRunning()
{
  if(target_found_.load()){
    cancelNav(false);
    return BT::NodeStatus::SUCCESS;
  }

  //VISUAL SERVOING
  if (tracking_active_) {
      
      if (node_->now() - last_track_time_ > rclcpp::Duration(2, 0)) {
          RCLCPP_WARN(node_->get_logger(), "\033[1;31m[Explore] Target perso di vista. Riprendo l'esplorazione...\033[0m");
          tracking_active_ = false;
          phase_ = Phase::PLANNING;
          return BT::NodeStatus::RUNNING;
      }

      
      if (nav_active_ || nav_request_in_flight_.load() || plan_request_in_flight_.load()) {
          cancelNav(true);
      }

      // Calcoliamo una posizione a 1.0 metro di distanza DAVANTI al rover, virata dell'angolo di errore suggerito dal detector.
      double rx, ry, ryaw;
      if (getRobotPose(rx, ry, ryaw)) {
          double approach_dist = 1.0; 
          double target_yaw = ryaw + target_yaw_error_;

          candidates_.clear();
          candidates_.push_back({
              rx + approach_dist * std::cos(target_yaw),
              ry + approach_dist * std::sin(target_yaw),
              target_yaw,
              approach_dist
          });
          cand_idx_ = 0;
          phase_ = Phase::NAVIGATING;
          (void)sendNavGoal();
          RCLCPP_INFO(node_->get_logger(), "\033[1;35m[Explore] INSEGUIMENTO ATTIVO! Correggo la rotta di %.2f rad\033[0m", target_yaw_error_);
      }
      return BT::NodeStatus::RUNNING;
  }

  if(!plan_client_ || !plan_client_->action_server_is_ready() ||
     !nav_client_  || !nav_client_->action_server_is_ready()){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[Explore] server non pronti, attendo...");
    return BT::NodeStatus::RUNNING;
  }

  if(!haveMap()){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[Explore] in attesa della /map...");
    return BT::NodeStatus::RUNNING;
  }

  if(phase_ == Phase::PLANNING){
    if(!plan_request_in_flight_.load()){
      if(cand_idx_ >= candidates_.size()){
        double rx, ry, ryaw;
        if(getRobotPose(rx, ry, ryaw) && buildRayFrontierCandidates(rx, ry)){
          cand_idx_ = 0;
        } else {
          return BT::NodeStatus::RUNNING;
        }
      }
      (void)sendPlanRequest();
    }

    if(plan_result_requested_){
      auto st = plan_result_future_.wait_for(1ms);
      if(st == std::future_status::ready){
        auto wrapped = plan_result_future_.get();
        plan_result_requested_ = false;
        plan_request_in_flight_.store(false);

        if(wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
           wrapped.result && wrapped.result->path.poses.size() >= static_cast<size_t>(min_path_poses_))
        {
          phase_ = Phase::NAVIGATING;
          (void)sendNavGoal();
        } else {
          cand_idx_++;
          return BT::NodeStatus::RUNNING;
        }
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  if(phase_ == Phase::NAVIGATING){
    if(!nav_active_ && !nav_request_in_flight_.load()){
      (void)sendNavGoal();
    }

    
    if(nav_active_ && (node_->now() - last_progress_time_ > progress_timeout_)){
      RCLCPP_WARN(node_->get_logger(), "[Explore] no progress, cancel & re-plan");
      cancelNav(true);
      if(++failures_ > max_failures_){
        RCLCPP_ERROR(node_->get_logger(), "[Explore] troppi fallimenti (%d)", failures_);
        return BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::PLANNING;
      return BT::NodeStatus::RUNNING;
    }

    if(nav_active_ && (node_->now() - nav_start_time_ > goal_timeout_)){
      RCLCPP_WARN(node_->get_logger(), "[Explore] timeout, cancel & re-plan");
      cancelNav(true);
      if(++failures_ > max_failures_){
        RCLCPP_ERROR(node_->get_logger(), "[Explore] troppi fallimenti (%d)", failures_);
        return BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::PLANNING;
      return BT::NodeStatus::RUNNING;
    }

    if(nav_result_requested_){
      auto st = nav_result_future_.wait_for(1ms);
      if(st == std::future_status::ready){
        auto wrapped = nav_result_future_.get();
        nav_result_requested_ = false;
        nav_active_ = false;
        nav_handle_.reset();

        if(wrapped.code == rclcpp_action::ResultCode::SUCCEEDED){
          failures_ = 0;
          cand_idx_ = candidates_.size(); 
          phase_ = Phase::PLANNING;
        } else {
          if(++failures_ > max_failures_){
            RCLCPP_ERROR(node_->get_logger(), "[Explore] troppi fallimenti (%d)", failures_);
            return BT::NodeStatus::FAILURE;
          }
          phase_ = Phase::PLANNING;
        }
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void ExploreUntilTargetFound::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[Explore] HALTED -> cancel nav");
  cancelNav(false);
}

bool ExploreUntilTargetFound::toMapFrame(double x_in, double y_in, double& x_map, double& y_map)
{
  std::lock_guard<std::recursive_mutex> lk(map_mtx_);
  if(!map_) return false;
  if(map_frame_.empty() || map_frame_ == frame_id_){
    x_map = x_in; y_map = y_in; return true;
  }
  geometry_msgs::msg::PoseStamped ps, out;
  ps.header.frame_id = frame_id_;

  //ps.header.stamp = node_->now();
  ps.header.stamp.sec = 0;          
  ps.header.stamp.nanosec = 0;

  ps.pose.position.x = x_in;
  ps.pose.position.y = y_in;
  ps.pose.orientation.w = 1.0;
  try{
    out = tf_buffer_->transform(ps, map_frame_, 100ms);
    x_map = out.pose.position.x;
    y_map = out.pose.position.y;
    return true;
  } catch(const std::exception&){
    return false;
  }
}

bool ExploreUntilTargetFound::insideMapBounds(double x, double y, double margin_m) const
{
  std::lock_guard<std::recursive_mutex> lk(map_mtx_);
  if(!map_) return true;
  const double min_x = origin_x_ + margin_m;
  const double min_y = origin_y_ + margin_m;
  const double max_x = origin_x_ + res_ * static_cast<double>(width_)  - margin_m;
  const double max_y = origin_y_ + res_ * static_cast<double>(height_) - margin_m;
  return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
}

bool ExploreUntilTargetFound::worldToMap(double x, double y, int& mx, int& my) const
{
  if(!map_) return false;
  const double dx = x - origin_x_;
  const double dy = y - origin_y_;
  if(dx < 0.0 || dy < 0.0) return false;
  mx = static_cast<int>(std::floor(dx / res_));
  my = static_cast<int>(std::floor(dy / res_));
  if(mx < 0 || my < 0 || mx >= static_cast<int>(width_) || my >= static_cast<int>(height_)) return false;
  return true;
}

int ExploreUntilTargetFound::cellAt(int mx, int my) const
{
  if(!map_) return -1;
  size_t idx = static_cast<size_t>(my) * width_ + static_cast<size_t>(mx);
  if(idx >= map_->data.size()) return -1;
  return map_->data[idx];
}

bool ExploreUntilTargetFound::isFreeCell(int mx, int my) const
{
  int v = cellAt(mx, my);
  if(v < 0) return false;           
  return v <= free_threshold_;      
}

bool ExploreUntilTargetFound::clearanceOKCells(int mx, int my, int min_cells) const
{
  if(!map_) return true;
  for(int dy=-min_cells; dy<=min_cells; ++dy){
    for(int dx=-min_cells; dx<=min_cells; ++dx){
      int nx = mx+dx, ny = my+dy;
      if(nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) return false;
      int v = cellAt(nx, ny);
      if(v < 0) return false;             
      if(v > free_threshold_) return false; 
    }
  }
  return true;
}

bool ExploreUntilTargetFound::getRobotPose(double& x, double& y, double& yaw)
{
  try{
    auto tf = tf_buffer_->lookupTransform(frame_id_, base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;

    const auto& q = tf.transform.rotation;
    double siny = 2.0*(q.w*q.z + q.x*q.y);
    double cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    yaw = std::atan2(siny, cosy);
    return true;
  } catch(const std::exception& e){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[Explore] TF %s->%s non disponibile: %s",
      frame_id_.c_str(), base_frame_.c_str(), e.what());
    return false;
  }
}

bool ExploreUntilTargetFound::buildRayFrontierCandidates(double rx_world, double ry_world)
{
  candidates_.clear();

  double rx_m=0.0, ry_m=0.0;
  if(!toMapFrame(rx_world, ry_world, rx_m, ry_m)) return false;

  const double two_pi = 2.0*M_PI;
  for(int i=0; i<n_rays_; ++i){
    double ang = (two_pi * i) / static_cast<double>(n_rays_);
    double gx_w=0.0, gy_w=0.0, yaw_w=ang;

    if(rayToFrontier(rx_m, ry_m, ang, gx_w, gy_w, yaw_w)){
      if(insideMapBounds(gx_w, gy_w, map_margin_m_)){
        
        
        // Controllo sulla Griglia di Esplorazione
        bool in_visited = false;
        if (explored_map_) {
            double res = explored_map_->info.resolution;
            double ox = explored_map_->info.origin.position.x;
            double oy = explored_map_->info.origin.position.y;
            
            int mx = (gx_w - ox) / res;
            int my = (gy_w - oy) / res;
            
            if (mx >= 0 && mx < (int)explored_map_->info.width && my >= 0 && my < (int)explored_map_->info.height) {
                int index = my * explored_map_->info.width + mx;
                if (explored_map_->data[index] >= 50) { 
                    in_visited = true;
                }
            }
        }
        
        if(!in_visited) {
          double d = std::hypot(gx_w - rx_world, gy_w - ry_world);
          candidates_.push_back({gx_w, gy_w, yaw_w, d});
        }
      }
    }
  }

  // Se i raggi normali si sono schiantati sui muri virtuali neri
  if(candidates_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "\033[1;33m[Explore] Zona locale esplorata! Scansione globale a lungo raggio...\033[0m");
    
    // Aumentiamo i tentativi a 500 e la distanza fino a 40 metri per scavalcare le enormi macchie nere!
    for(int i=0; i<500; ++i){
      double rand_ang = std::uniform_real_distribution<double>(0, two_pi)(rng_);
      double rand_dist = std::uniform_real_distribution<double>(2.0, 40.0)(rng_); // Raggio enorme
      double gx_w = rx_world + rand_dist * std::cos(rand_ang);
      double gy_w = ry_world + rand_dist * std::sin(rand_ang);

      if(insideMapBounds(gx_w, gy_w, map_margin_m_)){
        int gmx, gmy;
        // Controlliamo che il punto sia fisicamente libero dai muri
        if(worldToMap(gx_w, gy_w, gmx, gmy) && isFreeCell(gmx, gmy) && clearanceOKCells(gmx, gmy, clearance_cells_min_)){
          
          bool in_visited = false;
          // Controlliamo che il punto sia inesplorata
          if (explored_map_) {
              double res = explored_map_->info.resolution;
              int mx = (gx_w - explored_map_->info.origin.position.x) / res;
              int my = (gy_w - explored_map_->info.origin.position.y) / res;
              
              if (mx >= 0 && mx < (int)explored_map_->info.width && my >= 0 && my < (int)explored_map_->info.height) {
                  int index = my * explored_map_->info.width + mx;
                  if (explored_map_->data[index] >= 50) {
                      in_visited = true; 
                  }
              }
          }
          
          
          if(!in_visited) {
            candidates_.push_back({gx_w, gy_w, rand_ang, rand_dist});
          }
        }
      }
    }
  }


  std::sort(candidates_.begin(), candidates_.end(),
            [](const Candidate& a, const Candidate& b){ return a.dist < b.dist; });

  cand_idx_ = 0;
  return !candidates_.empty();
}

bool ExploreUntilTargetFound::rayToFrontier(double rx_m, double ry_m, double ang_m,
                                            double& goal_x_world, double& goal_y_world, double& yaw_world)
{
  std::lock_guard<std::recursive_mutex> lk(map_mtx_);
  if(!map_) return false;

  const double step = res_;
  const int max_steps = std::max(1, static_cast<int>(std::floor(max_scan_range_m_ / step)));

  double last_free_x = rx_m, last_free_y = ry_m;
  bool have_last_free = false;

  for(int s=1; s<=max_steps; ++s){
    double x = rx_m + s * step * std::cos(ang_m);
    double y = ry_m + s * step * std::sin(ang_m);

    if(!insideMapBounds(x, y, 0.0)) break;

    int mx, my;
    if(!worldToMap(x, y, mx, my)) break;

    int v = cellAt(mx, my);
    if(v < 0){
      if(have_last_free){
        double gx = last_free_x;
        double gy = last_free_y;

        gx -= backoff_m_ * std::cos(ang_m);
        gy -= backoff_m_ * std::sin(ang_m);

        if(!insideMapBounds(gx, gy, map_margin_m_)) return false;

        int gmx, gmy;
        if(!worldToMap(gx, gy, gmx, gmy)) return false;
        if(!isFreeCell(gmx, gmy)) return false;
        if(!clearanceOKCells(gmx, gmy, clearance_cells_min_)) return false;

        if(map_frame_ != frame_id_){
          geometry_msgs::msg::PoseStamped ps_in, ps_out;
          ps_in.header.frame_id = map_frame_;
          //ps_in.header.stamp = node_->now();

          ps_in.header.stamp.sec = 0;           
          ps_in.header.stamp.nanosec = 0;

          ps_in.pose.position.x = gx; ps_in.pose.position.y = gy; ps_in.pose.orientation.w = 1.0;
          try{
            ps_out = tf_buffer_->transform(ps_in, frame_id_, 100ms);
            goal_x_world = ps_out.pose.position.x;
            goal_y_world = ps_out.pose.position.y;
          } catch(const std::exception&){
            return false;
          }
        } else {
          goal_x_world = gx;
          goal_y_world = gy;
        }

        yaw_world = std::atan2(std::sin(ang_m), std::cos(ang_m));
        return true;
      } else {
        return false;
      }
    } else if(v <= free_threshold_) {
      last_free_x = origin_x_ + (mx + 0.5) * res_;
      last_free_y = origin_y_ + (my + 0.5) * res_;
      have_last_free = true;
    } else {
      return false;
    }
  }
  return false;
}

bool ExploreUntilTargetFound::sendPlanRequest()
{
  if(plan_request_in_flight_.load()) return true;

  if(candidates_.empty() || cand_idx_ >= candidates_.size()){
    return false;
  }

  double rx, ry, ryaw;
  if(!getRobotPose(rx, ry, ryaw)) return false;

  geometry_msgs::msg::PoseStamped start, goal;
  start.header.stamp = node_->now();
  start.header.frame_id = frame_id_;
  start.pose.position.x = rx;
  start.pose.position.y = ry;
  start.pose.orientation = yawToQuat(ryaw);

  const auto& C = candidates_[cand_idx_];
  goal.header.stamp = node_->now();
  goal.header.frame_id = frame_id_;
  goal.pose.position.x = C.x;
  goal.pose.position.y = C.y;
  goal.pose.orientation = yawToQuat(C.yaw);

  PlanAction::Goal g;
  g.start = start;
  g.goal  = goal;

  plan_request_in_flight_.store(true);

  auto opts = rclcpp_action::Client<PlanAction>::SendGoalOptions();
  opts.goal_response_callback = [this](typename PlanHandle::SharedPtr handle){
    plan_handle_ = handle;
    if(!plan_handle_){
      plan_request_in_flight_.store(false);
      RCLCPP_WARN(node_->get_logger(), "[Explore] plan REJECTED (cand #%zu)", cand_idx_);
      cand_idx_++;
    } else {
      plan_result_future_ = plan_client_->async_get_result(plan_handle_);
      plan_result_requested_ = true;
    }
  };
  opts.result_callback = [this](const typename PlanHandle::WrappedResult&){};

  (void)plan_client_->async_send_goal(g, opts);
  return true;
}

bool ExploreUntilTargetFound::sendNavGoal()
{
  if(nav_request_in_flight_.load() || nav_active_) return true;
  if(candidates_.empty() || cand_idx_ >= candidates_.size()) return false;

  const auto& C = candidates_[cand_idx_];

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.header.frame_id = frame_id_;
  goal.pose.position.x = C.x;
  goal.pose.position.y = C.y;
  goal.pose.orientation = yawToQuat(C.yaw);

  last_goal_x_ = C.x; last_goal_y_ = C.y; last_goal_yaw_ = C.yaw;

  NavAction::Goal g;
  g.pose = goal;

  nav_request_in_flight_.store(true);

  auto opts = rclcpp_action::Client<NavAction>::SendGoalOptions();
  opts.goal_response_callback = [this](typename NavHandle::SharedPtr handle){
    nav_request_in_flight_.store(false);
    nav_handle_ = handle;
    if(!nav_handle_){
      RCLCPP_WARN(node_->get_logger(), "[Explore] nav goal REJECTED (cand #%zu)", cand_idx_);
      cand_idx_++;
    } else {
      nav_active_ = true;
      nav_result_requested_ = true;
      nav_result_future_ = nav_client_->async_get_result(nav_handle_);
      nav_start_time_ = node_->now();
      last_feedback_dist_ = std::numeric_limits<double>::infinity();
      last_progress_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(),
        "\033[1;36m[Explore] NAV → (%.2f, %.2f) yaw=%.2f  [cand #%zu]\033[0m",
        last_goal_x_, last_goal_y_, last_goal_yaw_, cand_idx_);
    }
  };
  opts.feedback_callback = [this](typename NavHandle::SharedPtr,
                                  const std::shared_ptr<const NavAction::Feedback> fb){
    if(std::isfinite(last_feedback_dist_)){
      double delta = last_feedback_dist_ - fb->distance_remaining;
      if(delta >= min_progress_m_){
        last_progress_time_ = node_->now();
      }
    } else {
      last_progress_time_ = node_->now();
    }
    last_feedback_dist_ = fb->distance_remaining;
  };
  opts.result_callback = [this](const typename NavHandle::WrappedResult&){};

  (void)nav_client_->async_send_goal(g, opts);
  return true;
}

void ExploreUntilTargetFound::cancelNav(bool wait)
{
  if(nav_handle_){
    try{
      nav_client_->async_cancel_goal(nav_handle_);
      if(wait) rclcpp::sleep_for(50ms);
    } catch(...) {}
    nav_handle_.reset();
  }
  nav_active_ = false;
  nav_result_requested_ = false;
  nav_request_in_flight_.store(false);
}

void ExploreUntilTargetFound::exploredCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  explored_map_ = msg;
}

void ExploreUntilTargetFound::trackCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // msg->z == 1.0 significa che il detector ci sta chiedendo di inseguire
    if (msg->z == 1.0) {
        target_yaw_error_ = msg->x; 
        target_dist_estimate_ = msg->y;
        tracking_active_ = true;
        last_track_time_ = node_->now();
    } 
    // msg->z == 2.0 significa che siamo arrivati!
    else if (msg->z == 2.0) {
        tracking_active_ = false;
        target_found_.store(true); 
    }
}