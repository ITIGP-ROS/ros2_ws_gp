# Manual override: the gamepad path, joy_node + xbox_override together.
#
# Started by ackermann-joystick.service, which udev binds to the pad's device unit, so
# this comes up when a pad is connected and goes away when it is unplugged. Nothing here
# arms the vehicle -- xbox_override refuses to engage until the operator squeezes LT
# fully once, which is deliberate and is the reason the service can auto-start safely.
#
# WHY THESE TWO ARE ONE LAUNCH FILE. xbox_override is useless without /joy, and joy_node
# on its own publishes /joy that nothing consumes -- leaving the top-priority twist_mux
# slot (150) empty, which is exactly the state where the operator cannot take control
# away from Nav2 except by killing the stack or cutting power.
#
# NO --rt-axis / --lt-axis ARGUMENTS ON PURPOSE. The pad layout differs by transport
# (USB: 8 axes, LT ax2 | Bluetooth: 6 axes, LT ax4) and getting LT wrong is not cosmetic:
# over Bluetooth ax2 rests at 0.0 and the (1-axis)/2 throttle transform reads that as HALF
# PRESSED, so the vehicle would creep in REVERSE for as long as it is engaged. The node
# resolves the map itself from (len(axes), len(buttons)) once /joy is live -- see
# PAD_MAPS in xbox_override.py -- which is correct for whichever transport actually
# appeared. Passing them from here would be guessing at launch time, before /joy exists.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_fwd = LaunchConfiguration('max_fwd')
    max_rev = LaunchConfiguration('max_rev')
    max_steer = LaunchConfiguration('max_steer')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        # autorepeat_rate 20 Hz matches the override's publish rate: the override's
        # failsafe is that it publishes continuously, so a /joy that only updates on
        # change would starve it whenever the operator holds a steady input.
        parameters=[{'device_name': 'Xbox Series X Controller',
                     'autorepeat_rate': 20.0,
                     'deadzone': 0.05}],
        output='screen',
    )

    # --device-checked asserts the pad mapping was verified on THIS machine. That was a
    # manual step when the indices had to be passed in by hand; the node now resolves them
    # from the live /joy message, and the board's two layouts are recorded in PAD_MAPS, so
    # the check is satisfied by construction here. It still refuses any topic other than
    # /cmd_vel_teleop -- /cmd_vel and /cmd_vel_stamped bypass twist_mux and the emergency
    # lock entirely.
    override_node = Node(
        package='ackermann_bringup',
        executable='xbox_override.py',
        name='xbox_override',
        arguments=['--device-checked',
                   '--max-fwd', max_fwd,
                   '--max-rev', max_rev,
                   '--max-steer', max_steer],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'max_fwd', default_value='0.20',
            description='forward cap m/s (0.20 = proven ground speed, half MPPI vx_max)'),
        DeclareLaunchArgument(
            'max_rev', default_value='0.10',
            description='reverse cap m/s'),
        DeclareLaunchArgument(
            'max_steer', default_value='0.286',
            description='steering cap rad (the Tiva firmware clamp)'),
        joy_node,
        override_node,
    ])
