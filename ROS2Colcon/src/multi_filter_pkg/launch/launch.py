from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py'
            )
        ),
        launch_arguments={'headless': 'False'}.items()
    )

    prompt_node = Node(
        package='multi_filter_pkg',
        executable='initialpose_prompt_node',
        name='initialpose_prompt_node',
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        prompt_node
    ])

