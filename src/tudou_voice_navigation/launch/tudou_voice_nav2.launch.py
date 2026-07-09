from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    nav2_launch_path = os.path.join(
        get_package_share_directory('tudou_navigation2'),
        'launch',
        'tudou_nav2.launch.py'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path)
    )

    voice_nav_node = Node(
        package='tudou_voice_navigation',
        executable='tudou_voice_nav2',
        name='tudou_voice_nav2',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baudrate': 115200}
        ]
    )

    ld.add_action(nav2_launch)
    ld.add_action(voice_nav_node)

    return ld