from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_filter_pkg')

    # 1. Nav2-Launch starten
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={'headless': 'False'}.items()
    )

    # 2. In neuem Terminal das Bash-Skript mit Benutzerabfrage öffnen
    initialpose_prompt = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            os.path.join(pkg_dir, 'scripts', 'initialpose_prompt.sh') + '; exec bash'
        ],
        shell=False
    )

    return LaunchDescription([
        nav2_launch,
        initialpose_prompt
    ])

