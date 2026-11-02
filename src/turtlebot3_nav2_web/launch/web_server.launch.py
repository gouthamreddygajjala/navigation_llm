#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('host', default='0.0.0.0')
    port = LaunchConfiguration('port', default='8000')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_host_cmd = DeclareLaunchArgument(
        'host', default_value='0.0.0.0',
        description='Host address to bind local web server'
    )
    declare_port_cmd = DeclareLaunchArgument(
        'port', default_value='8000',
        description='Port number for local web server'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )

    web_server_node = Node(
        package='turtlebot3_nav2_web',
        executable='web_server_node',
        name='nav2_web_server',
        output='screen',
        parameters=[{
            'host': host,
            'port': port,
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_host_cmd)
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(web_server_node)

    return ld
