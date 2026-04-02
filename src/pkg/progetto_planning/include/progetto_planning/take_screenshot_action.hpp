#ifndef TAKE_SCREENSHOT_ACTION_HPP_
#define TAKE_SCREENSHOT_ACTION_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class TakeScreenshot : public BT::SyncActionNode
{
public:
  TakeScreenshot(const std::string& name, const BT::NodeConfiguration& config);

  
  BT::NodeStatus tick() override;

  
  static BT::PortsList providedPorts();

private:
  bool photo_taken_ = false; 
};

#endif 