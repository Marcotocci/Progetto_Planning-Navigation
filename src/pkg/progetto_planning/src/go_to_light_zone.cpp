#include "progetto_planning/go_to_light_zone.hpp"

GoToLightZone::GoToLightZone(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("go_to_light_zone_node");
  
  
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
  
  light_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/light_zone_goal", 10, std::bind(&GoToLightZone::light_callback, this, std::placeholders::_1));

  
  battery_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/battery_status", 10, std::bind(&GoToLightZone::battery_callback, this, std::placeholders::_1));
}

void GoToLightZone::light_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (this->status() != BT::NodeStatus::RUNNING) {
    locked_goal_ = *msg;
    has_goal_ = true;
  }
}

void GoToLightZone::battery_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (msg->data > last_charge_) {
    is_charging_ = true;
  } else {
    is_charging_ = false;
  }
  last_charge_ = msg->data;
}

BT::NodeStatus GoToLightZone::onStart() {
  rclcpp::spin_some(node_);
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_goal_) {
    RCLCPP_WARN(node_->get_logger(), "Radar cieco! Attendo...");
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "\033[1;31m[EMERGENZA]\033[0m Navigo verso la luce a X:%.2f Y:%.2f", 
              locked_goal_.pose.position.x, locked_goal_.pose.position.y);
  
  locked_goal_.header.stamp.sec = 0;
  locked_goal_.header.stamp.nanosec = 0;

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = locked_goal_;

  
  client_->async_send_goal(goal);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToLightZone::onRunning() {
  rclcpp::spin_some(node_);
  std::lock_guard<std::mutex> lock(mutex_);
  
  
  if (last_charge_ <= 0.0) {
    if (goal_sent_) {
      RCLCPP_ERROR(node_->get_logger(), "\033[1;31m[SISTEMA OFFLINE]\033[0m Motori spenti. Missione Fallita.");
      client_->async_cancel_all_goals(); 
      goal_sent_ = false;
    }
    return BT::NodeStatus::FAILURE; 
  }
  
 
  if (is_charging_ && goal_sent_) {
    RCLCPP_INFO(node_->get_logger(), "\033[1;32m[SALVEZZA]\033[0m Luce rilevata dalle ruote! Freno e mi ricarico qui.");
    client_->async_cancel_all_goals(); 
    goal_sent_ = false;
  }

  return BT::NodeStatus::RUNNING;
}

void GoToLightZone::onHalted() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_sent_) {
    client_->async_cancel_all_goals();
    goal_sent_ = false;
  }
  RCLCPP_INFO(node_->get_logger(), "\033[1;36m[SBLOCCO TARGET]\033[0m Fine ricarica, riprendo l'esplorazione.");
}