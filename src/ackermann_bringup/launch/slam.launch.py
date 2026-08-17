import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_ackermann_bringup = get_package_share_directory('ackermann_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    default_params_file = os.path.join(pkg_ackermann_bringup, 'config', 'slam.yaml')
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
            
        slam_toolbox_launch
    ])
