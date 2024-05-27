#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot = get_package_share_directory('multibot')
    install_dir = get_package_prefix('multibot')
    gazebo_models_path = os.path.join(pkg_box_bot, 'models')
    world_directory = os.path.join(pkg_box_bot,'worlds','space.world')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =install_dir + "/share" + ':' + gazebo_models_path
    
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    
    
    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_box_bot, 'worlds', 'space.world')],
            description='SDF world file'),
        
        ExecuteProcess(
            cmd=['gazebo', [world_directory], '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
    ])