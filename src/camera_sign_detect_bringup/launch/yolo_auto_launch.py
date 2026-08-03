import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _read_baked_backend(pkg_share):
    try:
        with open(os.path.join(pkg_share, 'config', 'backend')) as f:
            return f.read().strip()
    except OSError:
        return 'onnx'


def generate_launch_description():

    pkg_share = get_package_share_directory('camera_sign_detect_bringup')

    # param files
    camera_params = os.path.join(pkg_share, 'params', 'camera_params.yaml')
    detector_params = os.path.join(pkg_share, 'params', 'detector_params.yaml')
    detector_trt_params = os.path.join(pkg_share, 'params', 'detector_params_trt.yaml')
    detector_onnx_params = os.path.join(pkg_share, 'params', 'detector_params_onnx.yaml')

    # models
    trt_model_path = os.path.join(pkg_share, 'models', 'best.engine')
    onnx_model_path = os.path.join(pkg_share, 'models', 'best.onnx')
    labels_path = os.path.join(pkg_share, 'models', 'classes.names')

    # Default inference backend is decided at build time (Yocto) via config/backend
    backend_arg = DeclareLaunchArgument(
        'inference_backend',
        default_value=_read_baked_backend(pkg_share),
        choices=['onnx', 'trt'],
        description='Inference backend: onnx (ros2_yolos_cpp) or trt (ros2_yolos_cpp_trt)',
    )
    is_trt = IfCondition(EqualsSubstitution(LaunchConfiguration('inference_backend'), 'trt'))
    is_onnx = IfCondition(EqualsSubstitution(LaunchConfiguration('inference_backend'), 'onnx'))

    detector_trt = ComposableNode(
        package='ros2_yolos_cpp_trt',
        plugin='ros2_yolos_cpp_trt::YolosDetectorNode',
        name='yolos_detector',
        parameters=[detector_params, detector_trt_params,
            {  # Override with computed paths
                'model_path': trt_model_path,
                'labels_path': labels_path,
            }],
        remappings=[('~/image_raw', '/image_raw')],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition=is_trt,
    )

    detector_onnx = ComposableNode(
        package='ros2_yolos_cpp',
        plugin='ros2_yolos_cpp::YolosDetectorNode',
        name='yolos_detector',
        parameters=[detector_params, detector_onnx_params,
            {  # Override with computed paths
                'model_path': onnx_model_path,
                'labels_path': labels_path,
            }],
        remappings=[('~/image_raw', '/image_raw')],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition=is_onnx,
    )

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
            detector_trt,
            detector_onnx,
            ComposableNode(
                package='camera_sign_detect_bringup',
                plugin='YoloClassCanNode',
                name='yolo_class_can',
                parameters=[os.path.join(pkg_share, 'params', 'can_params.yaml')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # Wait for container/node discovery (DDS can take several seconds),
    # then configure. Retry until the transition succeeds (max ~30 s).
    configure_detector = ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 1 30); do ros2 lifecycle set /yolos_detector configure && exit 0; sleep 1; done; exit 1'],
        output='screen',
    )

    # Activate after configure completes (OnExecutionComplete)
    activate_detector = ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 1 30); do ros2 lifecycle set /yolos_detector activate && exit 0; sleep 1; done; exit 1'],
        output='screen',
    )

    return LaunchDescription([
        backend_arg,
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
