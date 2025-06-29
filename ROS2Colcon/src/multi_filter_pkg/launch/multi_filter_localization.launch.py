from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_filter_pkg')
    map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    sdf_model_path = os.path.join(pkg_share, 'models', 'turtlebot3_burger.sdf')

    return LaunchDescription([
        # Gazebo Sim starten
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            arguments=['-r', sdf_model_path],
            output='screen'
        ),

        # Bridge für Standard-Topics (tf, odom, cmd_vel, etc.)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@gz.msgs.Clock@rosgraph_msgs/msg/Clock',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            output='screen'
        ),

        # Nav2 Localization mit AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/localization_launch.py'
            ]),
            launch_arguments={
                'map': map_path,
                'use_sim_time': 'true'
            }.items(),
        ),

        # Dein Filter-Node
        Node(
            package='multi_filter_pkg',
            executable='multi_filter_node',
            name='multi_filter_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Initialpose setzen (verzögert, damit AMCL bereit ist)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='multi_filter_pkg',
                    executable='initial_pose_publisher.py',
                    name='initial_pose_publisher',
                    output='screen'
                )
            ]
        )
    ])
