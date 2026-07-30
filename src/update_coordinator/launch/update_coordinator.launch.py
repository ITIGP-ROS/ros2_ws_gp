import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('update_coordinator')

    default_params_file = os.path.join(pkg_dir, 'config', 'update_coordinator.yaml')

    secoc_key_arg = DeclareLaunchArgument(
        'secoc_key',
        default_value='/etc/ota_secoc.key',
        description='Path to SecOC AES-128 key file',
    )
    can_iface_arg = DeclareLaunchArgument(
        'can_iface',
        default_value='can0',
        description='CAN interface name (e.g., can0, vcan0)',
    )
    state_dir_arg = DeclareLaunchArgument(
        'state_dir',
        default_value='/var/lib/update_coordinator',
        description='Directory for SecOC freshness counter state files',
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout_sec',
        default_value='300.0',
        description='Update timeout in seconds',
    )

    node = Node(
        package='update_coordinator',
        executable='update_coordinator',
        name='update_coordinator',
        output='screen',
        parameters=[default_params_file, {
            'secoc_key': LaunchConfiguration('secoc_key'),
            'can_iface': LaunchConfiguration('can_iface'),
            'state_dir': LaunchConfiguration('state_dir'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
        }],
    )

    return LaunchDescription([
        secoc_key_arg,
        can_iface_arg,
        state_dir_arg,
        timeout_arg,
        node,
    ])
