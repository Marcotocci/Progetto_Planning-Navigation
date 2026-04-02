#ifndef GO_TO_LIGHT_ZONE_HPP
#define GO_TO_LIGHT_ZONE_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mutex>

class GoToLightZone : public BT::StatefulActionNode
{
public:
  GoToLightZone(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr light_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;

  geometry_msgs::msg::PoseStamped locked_goal_;
  bool has_goal_ = false;
  bool is_charging_ = false;
  float last_charge_ = 100.0;
  bool goal_sent_ = false;

  std::mutex mutex_;

  void light_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg);
};

#endif