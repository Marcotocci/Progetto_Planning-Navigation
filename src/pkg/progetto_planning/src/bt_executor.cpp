#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "battery_level_check_condition.cpp"
#include "is_target_found_condition.cpp"
#include "navigate_to_goal.cpp"
#include "explore_until_found.cpp"


#include "progetto_planning/take_screenshot_action.hpp"
#include "progetto_planning/go_to_light_zone.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_executor_node");

  BT::BehaviorTreeFactory factory;

  
  factory.registerNodeType<BatteryLevelCheckCondition>("BatteryLevelCheck");
  factory.registerNodeType<IsTargetFound>("IsTargetFound");
  
  factory.registerNodeType<TakeScreenshot>("TakeScreenshot");
  factory.registerNodeType<GoToLightZone>("GoToLightZone");

  BT::NodeBuilder builder_explore = [node](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<ExploreUntilTargetFound>(name, config, node);
  };
  factory.registerBuilder<ExploreUntilTargetFound>("ExploreUntilTargetFound", builder_explore);

  BT::NodeBuilder builder_navigate = [node](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<NavigateToGoal>(name, config, node);
  };
  factory.registerBuilder<NavigateToGoal>("NavigateToGoal", builder_navigate);

  if (argc < 2) {
    RCLCPP_ERROR(node->get_logger(), "Errore: Devi passare il percorso del file XML come argomento!");
    return 1;
  }
  std::string xml_path = argv[1];

  RCLCPP_INFO(node->get_logger(), "Caricamento Behavior Tree da: %s", xml_path.c_str());

  try {
    auto tree = factory.createTreeFromFile(xml_path);
    RCLCPP_INFO(node->get_logger(), "Cervello BT caricato! Inizio missione lunare...");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      
      rclcpp::spin_some(node); 
      
      tree.tickRoot();
      rate.sleep();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Errore durante il caricamento del BT: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}