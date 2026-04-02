#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>

class BatteryLevelCheckCondition : public BT::ConditionNode
{
public:
  BatteryLevelCheckCondition(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config),
    battery_level_(100.0),    
    battery_threshold_(20.0), 
    in_ricarica_(false)       
  {
    node_ = rclcpp::Node::make_shared("battery_level_check_condition");
    battery_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/battery_status", 10,
      std::bind(&BatteryLevelCheckCondition::batteryCallback, this, std::placeholders::_1));

    spinner_ = std::thread([this]() { rclcpp::spin(node_); });
  }

  ~BatteryLevelCheckCondition()
  {
    rclcpp::shutdown();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    auto & clk = *node_->get_clock();

    
    if (!in_ricarica_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), clk, 1000,
        "\033[1;32mBattery OK: %.2f%%\033[0m", battery_level_);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), clk, 1000,
        "\033[1;33mIn Ricarica... Livello attuale: %.2f%%\033[0m", battery_level_);
    }

    
    if (battery_level_ <= battery_threshold_ && !in_ricarica_) {
      in_ricarica_ = true;
      RCLCPP_WARN(node_->get_logger(),
        "\033[1;31mBattery CRITICA: %.2f%%! Avvio protocollo ricarica.\033[0m", battery_level_);
    } 
    
    else if (battery_level_ >= 95.0 && in_ricarica_) {
      in_ricarica_ = false;
      RCLCPP_INFO(node_->get_logger(),
        "\033[1;32mBattery CARICA: %.2f%%! Riprendo la missione.\033[0m", battery_level_);
    }

    
    if (in_ricarica_) {
      return BT::NodeStatus::FAILURE; 
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    battery_level_ = msg->data;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  std::thread spinner_;

  float battery_level_;
  float battery_threshold_;
  bool in_ricarica_; 
};