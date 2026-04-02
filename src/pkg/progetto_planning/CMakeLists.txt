cmake_minimum_required(VERSION 3.8)
project(progetto_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tutte le dipendenze ROS e di sistema
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#Plugin Costmap

add_library(illumination_layer SHARED src/illumination_layer.cpp)
target_include_directories(illumination_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(illumination_layer rclcpp nav2_costmap_2d pluginlib nav_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)


#Illumination Publisher

add_executable(illumination_publisher_node src/illumination_publisher.cpp)
target_include_directories(illumination_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(illumination_publisher_node rclcpp nav_msgs tf2_ros geometry_msgs)
target_link_libraries(illumination_publisher_node ${OpenCV_LIBRARIES} yaml-cpp)


# Battery Simulator

add_executable(battery_simulator src/battery_simulator.cpp)
# ---> AGGIUNTO tf2_ros QUI <---
ament_target_dependencies(battery_simulator rclcpp std_msgs nav_msgs tf2_ros)



# BT executor + nodes

add_executable(bt_executor 
  src/bt_executor.cpp
  src/take_screenshot_action.cpp
  src/go_to_light_zone.cpp
)
target_include_directories(bt_executor PUBLIC include)
ament_target_dependencies(bt_executor
  rclcpp rclcpp_action behaviortree_cpp_v3 std_msgs geometry_msgs nav_msgs nav2_msgs tf2_ros tf2_geometry_msgs sensor_msgs cv_bridge
)
target_link_libraries(bt_executor ${OpenCV_LIBRARIES})

add_executable(target_detector src/target_detector.cpp)
ament_target_dependencies(target_detector rclcpp sensor_msgs std_msgs cv_bridge OpenCV geometry_msgs)
target_link_libraries(target_detector ${OpenCV_LIBRARIES})


add_executable(light_zone_manager src/light_zone_manager.cpp)
ament_target_dependencies(light_zone_manager rclcpp nav2_msgs geometry_msgs nav_msgs tf2_ros)

add_executable(exploration_mapper src/exploration_mapper.cpp)
ament_target_dependencies(exploration_mapper rclcpp nav_msgs tf2_ros tf2_geometry_msgs geometry_msgs)


# Install

install(TARGETS illumination_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  illumination_publisher_node
  battery_simulator
  bt_executor
  target_detector
  light_zone_manager
  exploration_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY behavior_tree/ DESTINATION share/${PROJECT_NAME}/behavior_tree)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_package()
