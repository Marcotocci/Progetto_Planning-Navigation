#include "progetto_planning/navigate_to_goal.hpp"

using namespace std::chrono_literals;

BT::NodeStatus NavigateToGoal::onStart()
{
  if(auto v=getInput<double>("goal_x"))   gx_ = v.value(); else return BT::NodeStatus::FAILURE;
  if(auto v=getInput<double>("goal_y"))   gy_ = v.value(); else return BT::NodeStatus::FAILURE;
  if(auto v=getInput<double>("goal_yaw")) gyaw_ = v.value(); 
  if(auto v=getInput<std::string>("frame_id"))     frame_id_ = v.value();
  if(auto v=getInput<std::string>("server_name"))  server_name_ = v.value();
  if(auto v=getInput<int>("goal_timeout_ms"))      goal_timeout_ = rclcpp::Duration(std::chrono::milliseconds(v.value()));

  if(!nav_client_){
    nav_client_ = rclcpp_action::create_client<NavAction>(node_, server_name_);
  }
  if(!nav_client_->wait_for_action_server(2s)){
    RCLCPP_WARN(node_->get_logger(), "[NavigateToGoal] server %s non pronto", server_name_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id_;
  goal.header.stamp = node_->now();
  goal.pose.position.x = gx_;
  goal.pose.position.y = gy_;
  goal.pose.orientation = yawToQuat(gyaw_);

  NavAction::Goal g; g.pose = goal;

  goal_sent_ = false;
  nav_result_requested_ = false;
  nav_handle_.reset();

  auto opts = rclcpp_action::Client<NavAction>::SendGoalOptions();
  opts.goal_response_callback = [this](typename NavHandle::SharedPtr handle){
    nav_handle_ = handle;
    if(!nav_handle_){
      RCLCPP_ERROR(node_->get_logger(), "[NavigateToGoal] GOAL REJECTED");
    } else {
      nav_result_future_ = nav_client_->async_get_result(nav_handle_);
      nav_result_requested_ = true;
      goal_sent_ = true;
      start_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(),
        "\033[1;35m[NavigateToGoal] Ritorno alla base X:%.2f Y:%.2f\033[0m", gx_, gy_);
    }
  };
  opts.result_callback = [this](const typename NavHandle::WrappedResult&){};

  nav_client_->async_send_goal(g, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToGoal::onRunning()
{
  if(!goal_sent_) return BT::NodeStatus::RUNNING;

  if(goal_timeout_.nanoseconds() > 0 && (node_->now() - start_time_) > goal_timeout_)
  {
    RCLCPP_WARN(node_->get_logger(), "[NavigateToGoal] TIMEOUT -> cancel");
    onHalted();
    return BT::NodeStatus::FAILURE;
  }

  if(nav_result_requested_){
    auto st = nav_result_future_.wait_for(1ms);
    if(st == std::future_status::ready){
      auto wrapped = nav_result_future_.get();
      nav_result_requested_ = false;

      if(wrapped.code == rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[NavigateToGoal] DESTINAZIONE RAGGIUNTA\033[0m");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(node_->get_logger(), "[NavigateToGoal] FAILURE");
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToGoal::onHalted()
{
  if(nav_handle_){
    try{ nav_client_->async_cancel_goal(nav_handle_); } catch(...) {}
    nav_handle_.reset();
  }
  nav_result_requested_ = false;
  goal_sent_ = false;
}