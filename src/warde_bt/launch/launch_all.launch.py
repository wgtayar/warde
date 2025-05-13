#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'models',
            'myWorld',
            'boxes_world.sdf'
        ),
        description='Full path to the world SDF'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'rviz',
            'sim.config.rviz'
        ),
        description='RViz config file'
    )

    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'launch_sim.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file')
        }.items()
    )

    nav_server = Node(
        package='robot_nav',
        executable='navigate_node',
        name='navigate_server',
        output='screen'
    )

    spawn_box = Node(
        package='spawn_tools',
        executable='spawn_box_node',
        name='spawn_box_node',
        output='screen'
    )
    spawn_beer = Node(
        package='spawn_tools',
        executable='spawn_beer_node',
        name='spawn_beer_node',
        output='screen'
    )

    bt_main = Node(
        package='warde_bt',
        executable='warde_bt_main',
        name='warde_bt_main',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        sim_time_arg,
        rviz_arg,
        robot_sim,
        nav_server,
        spawn_box,
        spawn_beer,
        bt_main,
    ])
