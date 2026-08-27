# Nav2 against the simulated vehicle, using THE VEHICLE'S OWN PARAMETER FILE.
#
# 🔴 THERE IS NO SIMULATION COPY OF nav2_amcl.yaml, AND THERE MUST NOT BE ONE.
#
# The file this launches is ackermann_bringup/config/nav2_amcl.yaml - the same 600-odd
# lines that run on the car, MPPI critics, Smac penalties, inflation radii, turning radius
# and all. That is the entire reason this package exists: a tuning change can be driven in
# simulation and then shipped, because it is not a change to something that resembles the
# vehicle's configuration, it is a change to the vehicle's configuration.
#
# A previous version of this package did keep its own nav2_amcl_sim.yaml. By the time it
# was compared against the vehicle's file the two had drifted apart in the parameters that
# matter most - min_turning_r 0.65 against 1.20, max_beams 60 against 120, inflation and
# every MPPI critic weight - so simulation results said nothing about the car. Copying that
# file forward would have carried the drift with it. It was not copied.
#
# WHAT IS REWRITTEN, and nothing else:
#   use_sim_time      by the included launch files, which already substitute it into every
#                     node in the file (see navigation_launch.py / localization_launch.py).
#   the two bt xml    absolute paths that cannot be written into a portable yaml.
#   amcl initial_pose to wherever the vehicle was actually spawned.
#
# ⚠️ x/y/yaw MUST MATCH sim.launch.py's x/y/yaw. nav2_amcl.yaml sets set_initial_pose:
# true precisely so an unattended stack comes up fully ACTIVE instead of half-dead (the
# reasoning is written out at length in that file). The pose it names is (0, 0, 0), which
# is where sim.launch.py spawns by default and is free space in arena.world - so the
# default case is consistent with no rewriting at all. Spawn somewhere else and pass the
# same pose here, or AMCL starts convinced the vehicle is somewhere it is not.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_bringup = get_package_share_directory('ackermann_bringup')
    pkg_simulation = get_package_share_directory('ackermann_simulation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    run_activate = LaunchConfiguration('activate')

    # bt_xml is overridable so a candidate behaviour tree can be tested the same way a
    # candidate params file is - without forking the vehicle's nav2_bt.xml. These two
    # substitutions are what make the tree take effect at all: the path cannot be written
    # into a portable yaml, so whatever is set here WINS over the params file, and a tree
    # named in params_file alone would be silently ignored.
    bt_xml = LaunchConfiguration('bt_xml')
    param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml,
        'default_nav_through_poses_bt_xml': bt_xml,
        # amcl's initial_pose.{x,y,yaw}. RewrittenYaml matches on key NAME anywhere in the
        # document, so these are only safe because 'x', 'y' and 'yaw' appear nowhere else
        # in nav2_amcl.yaml - checked, not assumed. If a future parameter introduces a
        # bare 'x' somewhere, this rewrite would silently hit it too.
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'yaw': LaunchConfiguration('yaw'),
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': configured_params,
            'autostart': autostart,
        }.items(),
    )

    # ackermann_bringup's navigation_launch.py, NOT nav2_bringup's. The difference is the
    # command routing: this one sends the controller's output to /cmd_vel_nav_smoothed and
    # the recovery behaviours to /cmd_vel_recovery, so both enter twist_mux at their own
    # priority and both sit under the emergency lock. nav2_bringup's publishes straight to
    # /cmd_vel, which on this vehicle would bypass the mux and the lock entirely.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items(),
    )

    # The vehicle runs this as ExecStartPost because the Smac planner's configure() can
    # overrun a timeout hardcoded in nav2_util, leaving the stack half-activated and
    # silently ignoring goals. Simulation worlds are small enough that lifecycle_manager
    # usually finishes on its own, in which case this is a no-op by design - it is kept
    # on because when it is NOT a no-op it is the difference between a stack that accepts
    # goals and one that quietly does not. It exits non-zero if controller_server,
    # planner_server and bt_navigator are not all active, so its exit code is a usable
    # check that the run is worth watching.
    activate_node = Node(
        package='ackermann_bringup',
        executable='activate.py',
        name='activator',
        condition=IfCondition(run_activate),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_simulation, 'maps', 'arena.yaml'),
            description='Map yaml. The default matches worlds/arena.world exactly - both '
                        'are generated from one geometry definition by tools/make_arena.py.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_bringup, 'config', 'nav2_amcl.yaml'),
            description="The VEHICLE'S Nav2 parameters. Overriding this is how you test a "
                        'candidate tuning; do not fork the file to do it.'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description="AMCL's initial pose x. Must equal sim.launch.py's x."),
        DeclareLaunchArgument('y', default_value='0.0', description="AMCL's initial pose y."),
        DeclareLaunchArgument('yaw', default_value='0.0', description="AMCL's initial pose yaw."),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=os.path.join(pkg_bringup, 'config', 'nav2_bt.xml'),
            description="The VEHICLE'S behaviour tree. Override to test a candidate tree; "
                        'note its <Timeout msec> caps how long ANY goal may take, and a '
                        'legitimate multi-point turn on this vehicle can run close to it.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Let lifecycle_manager bring the stack up.'),
        DeclareLaunchArgument(
            'activate', default_value='true',
            description='Run activate.py to finish any lifecycle transition '
                        'lifecycle_manager abandoned. Harmless when there is none.'),

        localization_launch,
        navigation_launch,
        TimerAction(period=20.0, actions=[activate_node]),
    ])
