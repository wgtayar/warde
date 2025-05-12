from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{
        "use_sim_time": True,
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("combined_robot_description"),
                "urdf",
                "bmw_ur5_gripper_combined.urdf.xacro"
            ]),
            " ",
            "ur_type:=ur5",
            " ",
            "name:=ur5",
            " ",
            "tf_prefix:=ur5_"
        ])
    }],
    output="screen"
),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                ])
            ])
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "bmw_ur5_combined",
                "-topic", "robot_description"
            ],
            output="screen"
        )
    ])