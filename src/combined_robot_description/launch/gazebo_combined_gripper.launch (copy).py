from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from gazebo_ros.actions import SpawnEntity


def generate_launch_description():
    # 1) Build the robot_description by processing the combined xacro
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('combined_robot_description'),
            'urdf',
            'bmw_ur5_gripper_combined.urdf.xacro'
        ]),
        ' ur_type:=ur5 name:=ur5 tf_prefix:=ur5_'
    ])

    return LaunchDescription([
        
        # 1) Publish fake joint_states via a GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 2) Publish TFs based on that robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        # 3) Launch Gazebo itself
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                ])
            ])
        ),

        # 4) Spawn our combined model without tearing down Gazebo when done
        SpawnEntity(
            entity_name='bmw_ur5_combined',
            topic='robot_description',
            output='screen'
        ),
    ])
