import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取各功能包的路径
    tudou_desc_dir = get_package_share_directory('tudou_description')
    
    # 2. 引入你之前写好的 URDF 节点启动文件（自带 robot_state_publisher）
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tudou_desc_dir, 'launch', 'display_robot.launch.py'))
    )
    
    # 3. 引入雷达驱动节点（以你的镭神X10为例，去薅雷达厂家的 launch）
    # 注意：确保雷达驱动里发布的 frame_id 和你 URDF 里的雷达连线一致
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lsn10_launch.py'))
    )

    # 4. 启动你之前打通的 STM32 串口桥接节点
    stm32_bridge_node = Node(
        package='tudou_bringup', # 换成你存放 stm32_bridge.py 的包名
        executable='stm32_bridge',    # 确保在 setup.py 里注册了可执行权限
        name='stm32_bridge',
        output='screen'
    )

    # 5. 引入系统自带的 slam_toolbox 异步建图节点
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false', # 真实物理小车一律用 false
            'slam_params_file': os.path.join(get_package_share_directory('tudou_bringup'), 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    return LaunchDescription([
        urdf_launch,
        lidar_launch,
        stm32_bridge_node,
        slam_launch
    ])