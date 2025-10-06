import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('hik_camera_ros2')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 1. 定义相机节点 
    hik_camera_node = Node(
        package='hik_camera_ros2',
        executable='hik_camera_node',
        parameters=[params_file]
    )

    # 2. 定义 Rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    # 3. 将所有要启动的节点都放入 LaunchDescription 中
    return LaunchDescription([
        hik_camera_node,
        rviz_node  
    ])
