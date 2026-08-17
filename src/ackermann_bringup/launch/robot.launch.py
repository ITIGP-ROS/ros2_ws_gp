import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    hardware = LaunchConfiguration('hardware')
    can_interface = LaunchConfiguration('can_interface')
    enc_counts_per_rev = LaunchConfiguration('enc_counts_per_rev')

    ackermann_bringup_directory = get_package_share_directory('ackermann_bringup')

    hardware_config_file = os.path.join(
        ackermann_bringup_directory, 'config', 'hardware.yaml'
    )
    with open(hardware_config_file, 'r') as f:
        hardware_config = yaml.safe_load(f)

    robot_controllers = os.path.join(
        ackermann_bringup_directory,'config','controllers.yaml'
    )

    twist_mux_topics = os.path.join(
        ackermann_bringup_directory, 'config', 'twist_mux_topics.yaml'
    )
    twist_mux_locks = os.path.join(
        ackermann_bringup_directory, 'config', 'twist_mux_locks.yaml'
    )

    road_classification_config = os.path.join(
        ackermann_bringup_directory, 'config', 'road_classification.yaml'
    )

    pkg_path = get_package_share_directory('ackermann_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    ackermann_description = Command(['xacro ', xacro_file,
                                     ' hardware:=', hardware,
                                     ' can_interface:=', can_interface,
                                     ' enc_counts_per_rev:=', enc_counts_per_rev])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ackermann_description, 'use_sim_time': False}],
        output='screen'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_topics, twist_mux_locks, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen'
    )

    emergency_stop_server = Node(
        package='ackermann_bringup',
        executable='emergency_stop_server',
        output='screen'
    )

    road_classification_node = Node(
        package='ackermann_bringup',
        executable='road_classification_node',
        parameters=[road_classification_config,
                    {'can_interface': can_interface}],
        output='screen'
    )

    twist_stamper_node = Node(
        package='ackermann_bringup',
        executable='twist_stamper.py',
        parameters=[{'use_sim_time': False, 'frame_id': 'base_footprint'}],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': ackermann_description},
            robot_controllers],
        # NO REMAP OF ~/tf_odometry ONTO /tf HERE -- ON PURPOSE.
        #
        # ekf_filter_node owns odom -> base_footprint (see ekf.yaml: world_frame: odom).
        # The controller is configured for the SAME edge (controllers.yaml odom_frame_id /
        # base_frame_id) and is only kept off it by enable_odom_tf: false, so remapping it
        # onto /tf added a publisher that never published anything.
        #
        # That silent third publisher was not harmless: rosbag2 adapts its subscription QoS
        # to whichever publishers it has discovered when it subscribes, so depending on
        # discovery order it could match the mute one and reject the two that carry every
        # actual transform -- recording /tf as ZERO messages while still listing it as a
        # recorded topic and exiting successfully. Seen in 2 of 5 drive-length recordings.
        remappings=[
            ('/ackermann_controller/reference', '/cmd_vel_stamped'),
        ]
    )

    ekf_config_file = os.path.join(
        ackermann_bringup_directory, 'config', 'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('/odometry/filtered', '/odom')] 
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_controller',
                   '--param-file', robot_controllers],
        output='screen',
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster',
                   '--param-file', robot_controllers],
        output='screen',
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware',
            default_value='true',
            description='true if using hardware'),
        DeclareLaunchArgument(
            'can_interface',
            default_value=hardware_config['can_interface'],
            description='CAN interface name (e.g., can0, vcan0)'),
        DeclareLaunchArgument(
            'enc_counts_per_rev',
            default_value=str(hardware_config['enc_counts_per_rev']),
            description='Encoder counts per wheel revolution'),
        rsp_node,
        twist_mux_node,
        emergency_stop_server,
        road_classification_node,
        ekf_node,
        TimerAction(
            period=3.0,
            actions=[controller_manager]
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[ackermann_controller_spawner, imu_broadcaster_spawner, twist_stamper_node],
            )
        ),
    ])