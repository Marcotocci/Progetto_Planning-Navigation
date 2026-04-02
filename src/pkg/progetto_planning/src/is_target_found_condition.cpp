#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <atomic>

class IsTargetFound : public BT::ConditionNode
{
public:
  IsTargetFound(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("is_target_found_condition");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/target_found", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        found_ = msg->data;
      });
    spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  }

  ~IsTargetFound()
  {
    rclcpp::shutdown();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  BT::NodeStatus tick() override
  {
    return found_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> found_{false};
  std::thread spin_thread_;
};