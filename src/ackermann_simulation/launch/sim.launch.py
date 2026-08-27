# The vehicle, in Gazebo Classic, wired exactly as ackermann_bringup/launch/robot.launch.py
# wires it on the Orin.
#
# WHAT "EXACTLY" MEANS HERE, because it is the whole point of this file. Every node below
# reads the SAME configuration file the vehicle reads - controllers.yaml, ekf.yaml,
# twist_mux_topics.yaml, twist_mux_locks.yaml - and the command path is the same chain of
# topics in the same order:
#
#   /cmd_vel_teleop (150) ─┐
#   /cmd_vel_recovery (100)├─ twist_mux ─ /cmd_vel ─ twist_stamper ─ /cmd_vel_stamped
#   /cmd_vel_nav_smoothed ─┘     │                                          │
#          (10)                  └─ locked by /emergency_lock (255)         ▼
#                                                        /ackermann_controller/reference
#
# so a change to the mux priorities, the emergency lock, the controller geometry or the
# EKF fusion is exercised here before it reaches the car. There is no simulation copy of
# any of those files to drift out of sync.
#
# THE FOUR DIFFERENCES FROM robot.launch.py, all of them forced:
#
#   1. use_sim_time is true everywhere, and the controller_manager gets it through
#      controllers_sim.yaml because it runs inside gzserver rather than as a node this
#      file launches (see control.xacro).
#   2. The controller_manager is NOT started here at all - the gazebo_ros2_control plugin
#      starts it when the model spawns. That inverts the startup order, which is why the
#      spawners hang off event handlers below instead of firing at launch.
#   3. imu_scale.py runs with k:=1.0. On the vehicle k is 1.148, a measured correction for
#      the Tiva gyro under-reporting rotation by ~14%. Gazebo's gyro has no such error, so
#      keeping 1.148 here would INTRODUCE a 14.8% yaw-rate error that does not exist on
#      the car and make every turn overshoot in simulation only.
#   4. road_classification_node is absent. It opens a SocketCAN interface and there is no
#      CAN bus behind a simulation; it is not in the navigation path.
#
# ⚠️ The Livox path is NOT the default - see sim_lidar below and sensors.xacro.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_bringup = get_package_share_directory('ackermann_bringup')
    pkg_description = get_package_share_directory('ackermann_description')
    pkg_simulation = get_package_share_directory('ackermann_simulation')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    sim_lidar = LaunchConfiguration('sim_lidar')
    sim_camera = LaunchConfiguration('sim_camera')
    imu_source = LaunchConfiguration('imu_source')
    use_rviz = LaunchConfiguration('rviz')

    use_livox = IfCondition(PythonExpression(['"', sim_lidar, '" == "livox"']))
    use_broadcaster_imu = IfCondition(PythonExpression(['"', imu_source, '" == "broadcaster"']))

    # ------------------------------------------------------------------ model
    # NOT a plain `xacro ...`. urdf_for_gazebo.py expands the same file and then strips
    # the XML comments, because gazebo_ros2_control hands the URDF to the
    # controller_manager as `--param robot_description:=<urdf>` and rcl parses that text
    # as YAML: a comment containing ': ' or ending in ':' makes the parse fail, gzserver
    # logs one truncated line and keeps running, and the controller_manager then never
    # starts at all. The full account is in that script's docstring. The vehicle is
    # unaffected - robot.launch.py still runs xacro directly and keeps every comment.
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    robot_description = Command([
        'python3 ', os.path.join(pkg_simulation, 'tools', 'urdf_for_gazebo.py'), ' ',
        xacro_file,
        ' hardware:=false',
        ' sim_lidar:=', sim_lidar,
        ' sim_camera:=', sim_camera])

    # ParameterValue(..., value_type=str) is not optional. A bare Command() substitution
    # makes launch_ros try to parse the expanded URDF as YAML, and it aborts the whole
    # launch with "Unable to parse the value of parameter robot_description as yaml"
    # before Gazebo is even started.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}],
        output='screen',
    )

    # ------------------------------------------------------------------ gazebo
    # turtlebot3_gazebo's worlds reference model:// URIs that only resolve if its model
    # directory is on GAZEBO_MODEL_PATH. arena.world needs none of this - it uses only
    # sun and ground_plane from the stock Gazebo database - but appending the path costs
    # nothing and makes world:=.../turtlebot3_house.world work without extra setup.
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    try:
        gazebo_model_path = os.pathsep.join(filter(None, [
            gazebo_model_path,
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')]))
    except Exception:
        pass

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items(),
    )

    # z is 0.05, not 0. The wheels have radius 0.0325 and base_footprint sits on the
    # ground plane, so spawning at exactly 0 puts the collision meshes in contact with
    # the floor on the first physics step and the solver launches the vehicle. Dropping
    # it a few centimetres lets it settle instead.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ackermann',
                   '-x', x_pose, '-y', y_pose, '-z', '0.05', '-Y', yaw],
        output='screen',
    )

    # ------------------------------------------------------------------ ros2_control
    # --controller-manager-timeout is generous because the controller_manager does not
    # exist until gzserver has loaded the model AND the plugin: the spawner has to
    # outwait Gazebo's startup, not just the service call.
    def spawner(name, condition=None):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name,
                       '--controller-manager-timeout', '120',
                       '--switch-timeout', '120'],
            condition=condition,
            output='screen',
        )

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    ackermann_controller_spawner = spawner('ackermann_controller')
    imu_broadcaster_spawner = spawner('imu_broadcaster', use_broadcaster_imu)

    # ------------------------------------------------------------------ command chain
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(pkg_bringup, 'config', 'twist_mux_topics.yaml'),
                    os.path.join(pkg_bringup, 'config', 'twist_mux_locks.yaml'),
                    {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    # On the vehicle the controller_manager is launched by robot.launch.py and remaps
    # /ackermann_controller/reference onto /cmd_vel_stamped. Here the controller_manager
    # belongs to gzserver and cannot be remapped from this file, so the same edge is made
    # from the other end: twist_stamper publishes straight onto the controller's
    # reference topic. Identical graph, opposite remap.
    twist_stamper_node = Node(
        package='ackermann_bringup',
        executable='twist_stamper.py',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_footprint'}],
        remappings=[('/cmd_vel_stamped', '/ackermann_controller/reference')],
        output='screen',
    )

    emergency_stop_server = Node(
        package='ackermann_bringup',
        executable='emergency_stop_server',
        parameters=[{'use_sim_time': True,
                     'restore_goal': True,
                     'goal_restore_delay_sec': 2.0}],
        output='screen',
    )

    # ------------------------------------------------------------------ odometry
    # k:=1.0 - see note 3 in the header. bias_samples is left at its default: the
    # vehicle is stationary while it spawns and settles, which is what the bias window
    # requires, and in simulation the estimate simply comes out at ~0.
    #
    # imu_source selects which publisher feeds it:
    #   broadcaster (default) - ros2_control's imu_broadcaster, i.e. the same topic and
    #                           the same node type as the vehicle. Nothing is remapped.
    #   gazebo                - libgazebo_ros_imu_sensor's /imu/data, bypassing
    #                           ros2_control entirely. Use it if GazeboSystem fails to
    #                           bind the IMU state interfaces; the EKF cannot tell the
    #                           difference, since both carry the same measurement.
    imu_scale_node = Node(
        package='ackermann_bringup',
        executable='imu_scale.py',
        parameters=[{'use_sim_time': True, 'k': 1.0}],
        remappings=[('/imu_broadcaster/imu',
                     PythonExpression(['"/imu_broadcaster/imu" if "', imu_source,
                                       '" == "broadcaster" else "/imu/data"']))],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg_bringup, 'config', 'ekf.yaml'),
                    {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odom')],
        output='screen',
    )

    # ------------------------------------------------------------------ livox extras
    # Only with sim_lidar:=livox. The Mid-360 simulation publishes a cloud and no scan,
    # so /scan has to be synthesised the same way lidar.launch.py does it on the vehicle.
    # These parameters are copied from that file deliberately: the flattening is part of
    # what is under test, so it must be the vehicle's flattening.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        condition=use_livox,
        remappings=[('cloud_in', '/mid360_PointCloud2'), ('scan', '/scan')],
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'livox',
            'transform_tolerance': 0.01,
            'min_height': -0.15,
            'max_height': 0.30,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.28,
            'range_max': 40.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_simulation, 'config', 'display_sim.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_simulation, 'worlds', 'arena.world'),
            description='World file. arena.world is sized for this vehicle and has a '
                        'matching map in maps/arena.yaml; any other world needs its own '
                        'map (see slam, in the package README).'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run gzclient. false leaves gzserver headless, which is both '
                        'faster and the only way to run this over a plain SSH session.'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='Spawn x. MUST match nav2_sim.launch.py x, which is where AMCL '
                        'is told the vehicle starts.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw, rad.'),
        DeclareLaunchArgument(
            'sim_lidar', default_value='2d',
            description='2d: a 360-ray scanner publishing /scan directly (default, cheap). '
                        'livox: simulate the Mid-360 point cloud and flatten it with '
                        'pointcloud_to_laserscan, as the vehicle does. Needs '
                        'ros2_livox_simulation, which is not in this workspace, and casts '
                        '200,000 rays per second against the 2d path\'s 3,600.'),
        DeclareLaunchArgument(
            'sim_camera', default_value='true',
            description='false drops the simulated camera. The largest single CPU saving '
                        'available; use it for navigation-only runs.'),
        DeclareLaunchArgument(
            'imu_source', default_value='broadcaster',
            description='broadcaster: ros2_control imu_broadcaster, as on the vehicle. '
                        'gazebo: libgazebo_ros_imu_sensor /imu/data, as a fallback.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),

        rsp_node,
        twist_mux_node,
        emergency_stop_server,
        ekf_node,
        gazebo,
        rviz_node,

        # Gazebo needs to be up before anything can be spawned into it, and the model
        # needs to exist before its controller_manager does. Hence the chain: timer ->
        # spawn -> joint_state_broadcaster -> everything that depends on the vehicle
        # publishing state.
        TimerAction(period=3.0, actions=[spawn_entity]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_controller_spawner,
                         imu_broadcaster_spawner,
                         twist_stamper_node,
                         imu_scale_node,
                         pointcloud_to_laserscan],
            )
        ),
    ])
