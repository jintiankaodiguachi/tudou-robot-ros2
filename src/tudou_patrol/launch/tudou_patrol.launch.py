import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    tudou_patrol_node = Node(
        package='tudou_patrol',
        executable='tudou_vlm_agent',
        name='tudou_vlm_agent',
        output='screen'
    )

    tudou_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tudou_navigation2'), 'launch', 'tudou_nav2.launch.py')
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false'
        }.items()
    )

    ld.add_action(tudou_patrol_node)
    ld.add_action(tudou_nav2_launch)
    ld.add_action(realsense_launch)

    return ld