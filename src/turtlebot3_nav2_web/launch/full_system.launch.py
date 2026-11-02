#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_tb3_nav2_web = get_package_share_directory('turtlebot3_nav2_web')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default=os.environ.get('TURTLEBOT3_MODEL', 'waffle'))
    headless = LaunchConfiguration('headless', default='false')
    host = LaunchConfiguration('host', default='0.0.0.0')
    port = LaunchConfiguration('port', default='8000')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model', default_value='waffle',
        description='TurtleBot3 model: burger, waffle, waffle_pi'
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Whether to run Gazebo in headless mode'
    )
    declare_host_cmd = DeclareLaunchArgument(
        'host', default_value='0.0.0.0',
        description='Web server host binding'
    )
    declare_port_cmd = DeclareLaunchArgument(
        'port', default_value='8000',
        description='Web server port'
    )

    # 1. Simulation
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2_web, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': model,
            'headless': headless,
        }.items()
    )

    # 2. Nav2 Navigation Stack (Delayed slightly by 4 seconds to give Gazebo time to spawn)
    navigation_cmd = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tb3_nav2_web, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'model': model,
                }.items()
            )
        ]
    )

    # 3. Web Server & Bridge
    web_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2_web, 'launch', 'web_server.launch.py')
        ),
        launch_arguments={
            'host': host,
            'port': port,
            'use_sim_time': use_sim_time,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_host_cmd)
    ld.add_action(declare_port_cmd)

    ld.add_action(simulation_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(web_server_cmd)

    return ld
