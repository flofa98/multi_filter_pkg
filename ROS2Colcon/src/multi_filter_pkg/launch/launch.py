from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_filter_pkg')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={'headless': 'False'}.items()
    )

    prompt = ExecuteProcess(
        cmd=[os.path.join(pkg_dir, 'scripts', 'initialpose_prompt.sh')],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        prompt
    ])


