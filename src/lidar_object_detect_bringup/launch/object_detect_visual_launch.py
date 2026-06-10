
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    
    params_file =  os.path.join(
        get_package_share_directory('lidar_object_detect_bringup'),
        'params',
        'default_params.yaml'
    )
    
    detection_node = launch_ros.actions.Node(
        package='lidar_object_detection',
        executable='lidar_object_detector_node',
        name='lidar_object_detector_node',
        output='screen',
        parameters=[params_file]
    )

    object3d_visualizer_node = launch_ros.actions.Node(
        package='object_visualization',
        executable='object3d_visualizer_node',
        name='object3d_visualizer_node',
        output='screen',
        parameters=[params_file]
    )

    return launch.LaunchDescription([detection_node, object3d_visualizer_node])