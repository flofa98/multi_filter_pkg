from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_model = 'burger'  # oder 'waffle', je nach deinem Setup

    # Pfade definieren
    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )

    nav2_localization = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    map_path = os.path.join(
        get_package_share_directory('multi_filter_pkg'),
        'maps',
        'map.yaml'
    )

    return LaunchDescription([
        # 1. Gazebo-Simulation mit TurtleBot3 starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch),
            launch_arguments={'model': tb3_model}.items()
        ),

        # 2. AMCL + Karte starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_localization),
            launch_arguments={'map': map_path}.items()
        ),

        # 3. Dein Filter-Node
        Node(
            package='multi_filter_pkg',
            executable='multi_filter_node',
            name='multi_filter_node',
            output='screen'
        ),

        # 4. Initialpose automatisch setzen (über eigenen Python-Node)
        Node(
            package='multi_filter_pkg',
            executable='initial_pose_publisher.py',
            name='initial_pose_publisher',
            output='screen'
        )
    ])
