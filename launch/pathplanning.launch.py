import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    bot_1_controller_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_1_controller.yaml')
    bot_1_bt_navigator_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_1_btnavigator.yaml')
    bot_1_planner_yaml = os.path.join(get_package_share_directory('multibot'), 'config','bot_1_planner_server.yaml')
    bot_1_recovery_yaml = os.path.join(get_package_share_directory('multibot'), 'config', 'bot_1_recovery.yaml')
    bot_1_wp_follower = os.path.join(get_package_share_directory('multibot'), 'config', 'bot_1_waypoints.yaml')
    
    bot_2_controller_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_2_controller.yaml')
    bot_2_bt_navigator_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_2_btnavigator.yaml')
    bot_2_planner_yaml = os.path.join(get_package_share_directory('multibot'), 'config','bot_2_planner_server.yaml')
    bot_2_recovery_yaml = os.path.join(get_package_share_directory('multibot'), 'config', 'bot_2_recovery.yaml')

    bot_3_controller_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_3_controller.yaml')
    bot_3_bt_navigator_yaml = os.path.join(get_package_share_directory('multibot'),'config','bot_3_btnavigator.yaml')
    bot_3_planner_yaml = os.path.join(get_package_share_directory('multibot'), 'config','bot_3_planner_server.yaml')
    bot_3_recovery_yaml = os.path.join(get_package_share_directory('multibot'), 'config', 'bot_3_recovery.yaml')
    
    return LaunchDescription([
        
        # B O T 1
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace='bot_1',
            name='controller_server',
            output='screen',
            parameters=[bot_1_controller_yaml]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace='bot_1',
            name='planner_server',
            output='screen',
            parameters=[bot_1_planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace='bot_1',
            name='behavior_server',
            parameters=[bot_1_recovery_yaml],
            output='screen'),
        
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            namespace='bot_1',
            name='waypoint_follower_node',
            output='screen',
            parameters=[bot_1_wp_follower]),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace='bot_1',
            name='bt_navigator',
            output='screen',
            parameters=[bot_1_bt_navigator_yaml]),
        
        # B O T 2
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace='bot_2',
            name='controller_server',
            output='screen',
            parameters=[bot_2_controller_yaml]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace='bot_2',
            name='planner_server',
            output='screen',
            parameters=[bot_2_planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace='bot_2',
            name='behavior_server',
            parameters=[bot_2_recovery_yaml],
            output='screen'),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace='bot_2',
            name='bt_navigator',
            output='screen',
            parameters=[bot_2_bt_navigator_yaml]),
        
        # B O T 3
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace='bot_3',
            name='controller_server',
            output='screen',
            parameters=[bot_3_controller_yaml]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace='bot_3',
            name='planner_server',
            output='screen',
            parameters=[bot_3_planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace='bot_3',
            name='behavior_server',
            parameters=[bot_3_recovery_yaml],
            output='screen'),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace='bot_3',
            name='bt_navigator',
            output='screen',
            parameters=[bot_3_bt_navigator_yaml]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['bot_1/planner_server', 
                                        'bot_1/controller_server',
                                        'bot_1/behavior_server',
                                        'bot_1/waypoint_follower_node',
                                        'bot_1/bt_navigator',
                                        'bot_2/planner_server', 
                                        'bot_2/controller_server',
                                        'bot_2/behavior_server',
                                        'bot_2/bt_navigator',
                                        'bot_3/planner_server', 
                                        'bot_3/controller_server',
                                        'bot_3/behavior_server',
                                        'bot_3/bt_navigator'
                                        ]}])
    ])