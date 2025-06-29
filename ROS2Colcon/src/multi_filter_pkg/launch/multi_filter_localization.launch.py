from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Pfad zur Karte
    map_path = os.path.join(
        get_package_share_directory('multi_filter_pkg'),
        'maps',
        'map.yaml'
    )

    # Pfad zur Nav2 localization_launch.py
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    return LaunchDescription([
        # Starte Karte + AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'map': map_path}.items()
        ),

        # Starte Node
        Node(
            package='multi_filter_pkg',
            executable='multi_filter_node',
            name='multi_filter_node',
            output='screen'
        )
    ])
