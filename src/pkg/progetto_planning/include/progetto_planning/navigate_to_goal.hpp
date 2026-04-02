#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>
#include <future>
#include <cmath>

class NavigateToGoal : public BT::StatefulActionNode {
public:
  using NavAction = nav2_msgs::action::NavigateToPose;
  using NavHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  NavigateToGoal(const std::string& name,
                 const BT::NodeConfiguration& cfg,
                 const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<double>("goal_x",  "target X"),
      BT::InputPort<double>("goal_y",  "target Y"),
      BT::InputPort<double>("goal_yaw", 0.0, "yaw (rad)"),
      BT::InputPort<std::string>("frame_id",    std::string("map"),              "global frame"),
      BT::InputPort<std::string>("server_name", std::string("navigate_to_pose"), "Nav2 action name"),
      BT::InputPort<int>("goal_timeout_ms",     120000,                          "per-goal timeout (ms)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  geometry_msgs::msg::Quaternion yawToQuat(double yaw){
    geometry_msgs::msg::Quaternion q;
    q.x=0; q.y=0;
    q.z = std::sin(yaw*0.5);
    q.w = std::cos(yaw*0.5);
    return q;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  typename NavHandle::SharedPtr  nav_handle_;
  std::shared_future<typename NavHandle::WrappedResult> nav_result_future_;
  bool nav_result_requested_{false};
  bool goal_sent_{false};

  rclcpp::Time start_time_;
  rclcpp::Duration goal_timeout_{0,0};

  std::string frame_id_{"map"};
  std::string server_name_{"navigate_to_pose"};
  double gx_{0.0}, gy_{0.0}, gyaw_{0.0};
};