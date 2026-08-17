import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_ackermann_bringup = get_package_share_directory('ackermann_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_map = os.path.join(pkg_ackermann_bringup, 'maps', 'reception.yaml')
    default_nav2_params_file = os.path.join(pkg_ackermann_bringup, 'config', 'nav2_amcl.yaml')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ackermann_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        
        localization_launch
    ])
