from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) Generate the robot_description parameter by calling xacro correctly
    robot_description_content = Command([
        "xacro",
        PathJoinSubstitution([
            FindPackageShare("combined_robot_description"),
            "urdf",
            "bmw_ur5_combined2.urdf.xacro"
        ])
    ])

    # 2) Include the standard Gazebo launch (with ROS2 plugins)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={"verbose": "true"}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "bmw_ur5_combined"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    return LaunchDescription([
        gazebo_launch,
        rsp,
        spawn_entity,
    ])
