from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_prompt = LaunchConfiguration('use_initialpose_prompt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_initialpose_prompt',
            default_value='true',
            description='GUI-Fenster zur Initialpose anzeigen?'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'tb3_simulation_launch.py'
                )
            ),
            launch_arguments={'headless': 'False'}.items()
        ),

        Node(
            package='initialpose_gui',
            executable='initialpose_prompt_node',
            name='initialpose_prompt_node',
            output='screen',
            condition=IfCondition(use_prompt)
        )
    ])
