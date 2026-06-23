import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    pkg_share = get_package_share_directory('camera_sign_detect_bringup')
    
    # param files
    camera_params = os.path.join(pkg_share, 'params', 'camera_params.yaml')
    detector_params = os.path.join(pkg_share, 'params', 'detector_params.yaml')
    
    # model
    model_path = os.path.join(pkg_share, 'models', 'best.onnx')
    labels_path = os.path.join(pkg_share, 'models', 'classes.names')

    container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='v4l2_camera',
                plugin='v4l2_camera::V4L2Camera',
                name='camera',
                parameters=[camera_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='ros2_yolos_cpp',
                plugin='ros2_yolos_cpp::YolosDetectorNode',
                name='yolos_detector',
                parameters=[detector_params, # Load YAML first
                    {  # Override with computed paths
                        'model_path': model_path,
                        'labels_path': labels_path,
                    }],
                remappings=[ ( '~/image_raw', '/image_raw'), # important as its not a param in the node
                            ('~/detections', '~/detections'),
                            ('~/debug_image', '~/debug_image')],
                extra_arguments=[{'use_intra_process_comms': True}],
                
            ),
        ],
        output='screen',
    )

    # Wait for container (sleep 3 here), then configure
    configure_detector = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 3 && ros2 lifecycle set /yolos_detector configure'],
        output='screen',
    )

    # Activate after configure completes (OnExecutionComplete)
    activate_detector = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/yolos_detector', 'activate'],
        output='screen',
    )

    return LaunchDescription([
        container,
        RegisterEventHandler(
            OnProcessStart(
                target_action=container,
                on_start=[
                    LogInfo(msg='Container started, waiting to configure detector...'),
                    configure_detector,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=configure_detector,
                on_completion=[
                    LogInfo(msg='Configure complete, activating detector...'),
                    activate_detector,
                ],
            )
        )
    ])