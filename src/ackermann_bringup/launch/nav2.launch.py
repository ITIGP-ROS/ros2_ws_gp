import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_ackermann_bringup = get_package_share_directory('ackermann_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    default_nav2_params_file = os.path.join(pkg_ackermann_bringup, 'config', 'nav2_amcl.yaml')
    nav2_params_file = LaunchConfiguration('params_file')
    
    # For custom behavior true inclusion without absolute path
    param_substitutions = {
        'default_nav_to_pose_bt_xml': os.path.join(
            pkg_ackermann_bringup, 'config', 'nav2_bt.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            pkg_ackermann_bringup, 'config', 'nav2_bt.xml')
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ackermann_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        
        nav2_launch
    ])

