import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_yaml_1 = os.path.join(get_package_share_directory('multibot'), 'config','bot_1_amcl.yaml')
    nav2_yaml_2 = os.path.join(get_package_share_directory('multibot'), 'config','bot_2_amcl.yaml')
    nav2_yaml_3 = os.path.join(get_package_share_directory('multibot'), 'config','bot_3_amcl.yaml')
    map_file = os.path.join(get_package_share_directory('multibot'), 'maps', 'multibot_map.yaml')
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace='bot_1',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_1]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace='bot_2',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_2]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace='bot_3',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_3]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server','bot_1/amcl','bot_2/amcl','bot_3/amcl']}]
        )
    ])
