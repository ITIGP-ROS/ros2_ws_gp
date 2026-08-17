import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    compressor_node = Node(
        package='ackermann_bringup',
        executable='camera_compressor.py',
        name='camera_compressor',
        output='screen'
    )

    return LaunchDescription([
        compressor_node
    ])
