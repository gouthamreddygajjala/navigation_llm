#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

if 'TURTLEBOT3_MODEL' not in os.environ:
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'


def launch_setup(context, *args, **kwargs):
    pkg_tb3_nav2_web = get_package_share_directory('turtlebot3_nav2_web')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model').perform(context)
    autostart = LaunchConfiguration('autostart')
    
    # Select parameter file based on model (waffle or burger)
    if 'waffle' in model:
        default_params = os.path.join(pkg_tb3_nav2_web, 'config', 'nav2_params_waffle.yaml')
    else:
        default_params = os.path.join(pkg_tb3_nav2_web, 'config', 'nav2_params_burger.yaml')

    # Resolve params_file, handling cases where other launch files set it to ''
    params_file_eval = ''
    try:
        params_file_eval = LaunchConfiguration('params_file').perform(context)
    except Exception:
        pass
    if not params_file_eval:
        params_file_eval = default_params

    # Resolve map file
    default_map = os.path.join(pkg_tb3_nav2_web, 'maps', 'house_map.yaml')
    map_file_eval = ''
    try:
        map_file_eval = LaunchConfiguration('map').perform(context)
    except Exception:
        pass
    if not map_file_eval:
        map_file_eval = default_map

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_eval,
            'use_sim_time': use_sim_time,
            'params_file': params_file_eval,
            'autostart': autostart,
            'use_composition': 'False',
        }.items()
    )

    return [nav2_bringup_cmd]


def generate_launch_description():
    pkg_tb3_nav2_web = get_package_share_directory('turtlebot3_nav2_web')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
        description='TurtleBot3 model: burger, waffle, waffle_pi'
    )
    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_tb3_nav2_web, 'maps', 'house_map.yaml'),
        description='Full path to map file to load'
    )
    declare_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg_tb3_nav2_web, 'config', 'nav2_params_waffle.yaml'),
        description='Full path to Nav2 parameter file'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically transition lifecycle nodes to active'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
