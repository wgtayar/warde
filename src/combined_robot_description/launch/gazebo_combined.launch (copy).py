#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import FindExecutable, Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1) Declare your two YAML args
    declare_controllers = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_simulation_gazebo'),
            'config',
            'ur_controllers.yaml'
        ]),
        description='Path to ur_controllers.yaml'
    )
    declare_initial = DeclareLaunchArgument(
        'initial_positions_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'),
            'config',
            'initial_positions.yaml'
        ]),
        description='Path to initial_positions.yaml'
    )

    # 2) Grab them
    controllers_file       = LaunchConfiguration('controllers_file')
    initial_positions_file = LaunchConfiguration('initial_positions_file')

    # 3) Build robot_description via xacro (we hard-code sim_gazebo inside the xacro itself)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('combined_robot_description'),
        'urdf',
        'bmw_ur5_combined.urdf.xacro'
    ])
    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'controllers_file:=',       controllers_file,       ' ',
        'initial_positions_file:=', initial_positions_file
    ])
    robot_description = {'robot_description': robot_description_cmd}

    # 4) Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # 5) Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    # 6) Spawn the two controllers
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster','--controller-manager','/controller_manager'],
        output='screen',
    )
    traj = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller','-c','/controller_manager'],
        output='screen',
    )

    # 7) Finally, spawn into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic','robot_description',
            '-entity','bmw_ur5_combined'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_controllers,
        declare_initial,
        gazebo,
        rsp,
        jsb,
        traj,
        spawn,
    ])
