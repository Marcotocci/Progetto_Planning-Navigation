import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    pkg_share_dir = get_package_share_directory('progetto_planning')
    
    bt_xml_path = os.path.join(pkg_share_dir, 'behavior_tree', 'lunar_mission.xml')

    #rviz_config_path = os.path.join(get_package_share_directory('progetto_planning'), 'rviz', 'lunar_mission.rviz')

    sim_time_param = {'use_sim_time': True}

    return LaunchDescription([
        
        Node(
            package='progetto_planning',
            executable='battery_simulator',
            name='battery_simulator',
            output='screen',
            parameters=[sim_time_param]
        ),

       
        Node(
            package='progetto_planning',
            executable='light_zone_manager',
            name='light_zone_manager',
            output='screen',
            parameters=[sim_time_param]
        ),

        
        Node(
            package='progetto_planning',
            executable='target_detector',
            name='target_detector',
            output='screen',
            parameters=[sim_time_param]
        ),

        
        Node(
            package='progetto_planning',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
            parameters=[sim_time_param],
            arguments=[bt_xml_path] 
        ),

        Node(
            package='progetto_planning',
            executable='exploration_mapper',
            name='exploration_mapper',
            output='screen',
            parameters=[sim_time_param]
        )

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_path],
        #     parameters=[{'use_sim_time': True}]
        # )
    ])