#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Safe fallback default in os.environ
if 'TURTLEBOT3_MODEL' not in os.environ:
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'


def clean_gazebo_model_paths(pkg_tb3_gazebo):
    """
    Sanitize GAZEBO_MODEL_PATH to ensure fast startup without scanning 500+ ROS packages.
    """
    home_models = os.path.expanduser('~/.gazebo/models')
    sys_models = '/usr/share/gazebo-11/models'
    tb3_models = os.path.join(pkg_tb3_gazebo, 'models')

    # Remove root /opt/ros/humble/share which depthai_descriptions injects
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    cleaned_existing = [
        p for p in existing.split(':')
        if p and p.rstrip('/') != '/opt/ros/humble/share'
    ]

    ordered_paths = [home_models, sys_models, tb3_models] + cleaned_existing
    valid_paths = []
    for p in ordered_paths:
        if p not in valid_paths and os.path.isdir(p):
            valid_paths.append(p)

    return ':'.join(valid_paths)


def launch_setup(context, *args, **kwargs):
    pkg_tb3_nav2_web = get_package_share_directory('turtlebot3_nav2_web')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model').perform(context)
    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    world_file = LaunchConfiguration('world').perform(context)

    # Sync environment variables
    os.environ['TURTLEBOT3_MODEL'] = model
    cleaned_model_path = clean_gazebo_model_paths(pkg_tb3_gazebo)
    os.environ['GAZEBO_MODEL_PATH'] = cleaned_model_path
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    os.environ['GAZEBO_RESOURCE_PATH'] = f"/usr/share/gazebo-11:{os.environ.get('GAZEBO_RESOURCE_PATH', '')}"

    # Paths to model SDF and URDF
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    urdf_path = os.path.join(pkg_tb3_gazebo, 'urdf', f'turtlebot3_{model}.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Gazebo Server (starts master & physics)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo Client (GUI) - delayed by 1.2s to ensure gzserver has bound master socket
    gzclient_cmd = TimerAction(
        period=1.2,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                condition=UnlessCondition(headless)
            )
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Spawn Entity in Gazebo (delayed slightly to ensure physics world loaded)
    spawn_entity_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                arguments=[
                    '-entity', f'turtlebot3_{model}',
                    '-file', sdf_path,
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    return [
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_node
    ]


def generate_launch_description():
    pkg_tb3_nav2_web = get_package_share_directory('turtlebot3_nav2_web')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )
    declare_model_cmd = DeclareLaunchArgument(
        'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
        description='TurtleBot3 model: burger, waffle, waffle_pi'
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Whether to run Gazebo in headless mode (no gzclient GUI)'
    )
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial robot X coordinate in meters'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial robot Y coordinate in meters'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_tb3_nav2_web, 'worlds', 'obstacle_world.world'),
        description='Full path to world SDF/world file'
    )

    cleaned_model_path = clean_gazebo_model_paths(pkg_tb3_gazebo)

    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=cleaned_model_path
    )
    set_gazebo_db_uri_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    set_gazebo_resource_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=f"/usr/share/gazebo-11:{os.environ.get('GAZEBO_RESOURCE_PATH', '')}"
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_gazebo_db_uri_cmd)
    ld.add_action(set_gazebo_resource_path_cmd)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
