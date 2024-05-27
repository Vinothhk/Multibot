#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    robot_1_name = "bot_1"
    robot_2_name = "bot_2"
    robot_3_name = "bot_3"
    entity_name_1 = robot_1_name+"-"+str(int(random.random()*100000))
    entity_name_2 = robot_2_name+"-"+str(int(random.random()*100000))
    entity_name_3 = robot_3_name+"-"+str(int(random.random()*100000))

    spawn_robot_1 = Node(
        package='gazebo_ros',
        namespace='bot_1',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name_1,'-x', '5.0' , '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-topic', '/bot_1/robot_description'
                   ]
    )
    
    spawn_robot_2 = Node(
        package='gazebo_ros',
        namespace='bot_2',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name_2, '-x', '-5.0' , '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-topic', '/bot_2/robot_description'
                   ]
    )
    spawn_robot_3 = Node(
        package='gazebo_ros',
        namespace='bot_3',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name_3, '-x', '0.0' , '-y', '5.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-topic', '/bot_3/robot_description'
                   ]
    )

    return LaunchDescription(
        [
            spawn_robot_1,
            spawn_robot_2,
            spawn_robot_3
            ]
    )
