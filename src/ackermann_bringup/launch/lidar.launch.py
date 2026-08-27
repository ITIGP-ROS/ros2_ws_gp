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
            # 🔴 0.00436 (0.25 deg, 1442 bins) -> 0.01745 (1.0 deg, 360 bins).
            # Pairs with observation_persistence in nav2_amcl.yaml -- ship BOTH or
            # neither.
            #
            # The Mid-360 scans a NON-REPETITIVE rosette: it covers different bearings
            # every frame. Flattening that into 1442 fixed bins asks for far more angular
            # resolution than the per-frame point density supports, so the bins blink.
            # MEASURED, bag 20260826_165022_nav_195020, 1875 scans over 191 s:
            #     bins with a return in a typical scan   88.5%
            #     bins that ALWAYS have a return         0
            #     bins that NEVER have a return          0
            #     bins that come and go                  1442, i.e. ALL of them
            #     present/absent flips per bin           median 142, p95 896, max 969
            # Downstream that oscillates the costmap and the planner intermittently
            # refuses to start -- see the observation_persistence note in nav2_amcl.yaml
            # for the full chain and the 39 "Starting point in lethal space!" failures.
            #
            # 4x fewer bins puts ~4x the points in each, so bins stay occupied frame to
            # frame. Nothing real is lost: at obstacle_max_range 3.5 m, 1 deg subtends
            # 6 cm, which is about ONE cell of the 0.05 m costmap. AMCL is likewise
            # unaffected at laser_max_range 15.0.
            'angle_increment': 0.01745,
            'scan_time': 0.1,
            'range_min': 0.28,
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