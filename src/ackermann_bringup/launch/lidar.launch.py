import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Paths
    pkg_ackermann_bringup = get_package_share_directory('ackermann_bringup')
    
    # Livox config path (ip configuration)
    livox_config = os.path.join(pkg_ackermann_bringup, 'config', 'MID360s_config.json')
    
    # Livox parameters (Not related to ip configuration)
    livox_ros2_params = [

        {"publish_freq": 10.0},
        {"frame_id": 'lidar_link'},
        {"user_config_path": livox_config}
    ]

    # Livox driver node
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )


    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),   # Livox hardware output
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': 0.01,
            'min_height': -0.15,
            'max_height': 0.30,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 40.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )


    return LaunchDescription([
        livox_driver,
        pointcloud_to_laserscan,
    ])