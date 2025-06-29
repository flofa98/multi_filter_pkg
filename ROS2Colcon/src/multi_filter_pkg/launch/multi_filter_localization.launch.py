from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_model = 'burger'  

    # Pfade
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
        # Gazebo starten mit TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch),
            launch_arguments={'model': tb3_model}.items()
        ),

        # Nav2 Localization mit Karte + AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_localization),
            launch_arguments={'map': map_path}.items()
        ),

        # Filter-Node
        Node(
            package='multi_filter_pkg',
            executable='multi_filter_node',
            name='multi_filter_node',
            output='screen'
        ),

        # Initialpose setzen
        Node(
            package='geometry_msgs',
            executable='ros2',
            name='set_initial_pose',
            arguments=[
                'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped',
                '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}'
            ],
            output='screen'
        )
    ])
