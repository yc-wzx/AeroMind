"""Opt-in 3D sensors and LIO. Derived from frozen Stage2; no GT feedback."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('uav_bringup')
    planning = get_package_share_directory('uav_planning')
    sim = {'use_sim_time': True}
    world = os.path.join(bringup, 'worlds', 'provincial_stage2_lio_position_imu.sdf')
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '2', world],
                       output='log', additional_env={'QT_QPA_PLATFORM': 'xcb',
                           'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '4', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.path.join(get_package_share_directory('stage2_lio_sim'),'..','..','lib')})
    gui = ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '2', '--gui-config',
                 os.path.join(bringup, 'config', 'competition_gazebo_gui.config')],
            condition=IfCondition(LaunchConfiguration('gui')),
            output='log', additional_env={'QT_QPA_PLATFORM': 'xcb'})
    return LaunchDescription([
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('max_planar_speed_mps', default_value='0.25'),
        DeclareLaunchArgument('max_planar_acc_mps2', default_value='0.35'),
        DeclareLaunchArgument('max_yaw_rate_rad_s', default_value='0.30'),
        server, gui,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gazebo_bridge', output='screen', arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                 '/gazebo/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                 '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                 '/simulation/lidar3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                 '/simulation/imu3d@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                 '/simulation/imu_kinematics@std_msgs/msg/String[ignition.msgs.StringMsg',
                 '/model/omni_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
             ]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_identity', parameters=[sim], arguments=[
                 '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='stage2_lio_sim', executable='lio_sensor_adapter.py', parameters=[sim], output='screen'),
        Node(package='stage2_lio_sim', executable='lio_point_boundary_mapping.py', name='lio_mapping',
             parameters=[os.path.join(bringup,'config','spark_provincial_noise_model_sim.yaml'),sim], output='log',
             remappings=[('lidar','/lio/lidar'),('imu','/lio/imu'),('odometry','/lio/odometry'),
                         ('cloud_registered','/lio/cloud_registered'),('path','/lio/path')]),
        Node(package='stage2_lio_sim', executable='lio_odometry_adapter.py',parameters=[sim],output='screen'),
        Node(package='stage2_lio_sim', executable='lio_safe_navigation_interface.py',
             condition=IfCondition(LaunchConfiguration('navigation')),
             remappings=[('/gazebo/odometry', '/localization/lio_navigation_odometry')],
             name='gazebo_navigation_interface', output='screen', parameters=[sim, {
                 'field_config': os.path.join(planning, 'config', 'provincial_2025_provisional.json'),
                 'occupancy_map_pgm': os.path.join(bringup, 'maps', 'provincial_2025_provisional.pgm'),
                 'grid_route_clearance': 0.40,
                 'dynamic_goal_clearance': 0.40,
                 'imperfect_sensors': False,
                 'provincial_reference_route_mode': True,
                 'provincial_rect_guard': True,
                 'provincial_min_body_gap_m': 0.08,
                 'provincial_wall_thickness_m': 0.055,
                 'auto_goal': ParameterValue(LaunchConfiguration('auto_goal'), value_type=bool),
                 'goal_delay_sec': ParameterValue(LaunchConfiguration('goal_delay_sec'), value_type=float),
             }]),
        Node(package='ego_planner', executable='ego_planner_node',
             condition=IfCondition(LaunchConfiguration('navigation')),
             name='ego_planner_node', output='log',
             parameters=[os.path.join(bringup, 'config', 'ego_provincial_2025_provisional.yaml'), sim, {
                 'manager/max_vel': ParameterValue(LaunchConfiguration('max_planar_speed_mps'), value_type=float),
                 'optimization/max_vel': ParameterValue(LaunchConfiguration('max_planar_speed_mps'), value_type=float),
                 'manager/max_acc': ParameterValue(LaunchConfiguration('max_planar_acc_mps2'), value_type=float),
                 'optimization/max_acc': ParameterValue(LaunchConfiguration('max_planar_acc_mps2'), value_type=float),
                 'fsm/replan_from_odom_if_diverged': True,
                 'fsm/replan_odom_divergence_m': 0.12,
             }],
             remappings=[('odom_world','/ground/odometry'),
                         ('grid_map/odom','/ground/odometry'),
                         ('grid_map/cloud','/cloud_registered_2d')]),
        Node(package='uav_planning', executable='ego_goal_adapter.py',
             condition=IfCondition(LaunchConfiguration('navigation')),
             name='ego_goal_adapter', output='screen',
             parameters=[os.path.join(planning,'config','ego_goal_adapter.yaml'), sim,
                         {'input_topic': '/navigation/segment_goal'}]),
        Node(package='uav_planning', executable='ego_trajectory_executor.py',
             condition=IfCondition(LaunchConfiguration('navigation')),
             name='ego_trajectory_executor', output='screen',
             parameters=[os.path.join(planning,'config','ego_trajectory_executor.yaml'),
                         sim, {'enable_cmd_vel_output':True,
                               'trajectory_start_clock': 'ros',
                               'max_vel_x_mps': ParameterValue(LaunchConfiguration('max_planar_speed_mps'), value_type=float),
                               'max_vel_y_mps': ParameterValue(LaunchConfiguration('max_planar_speed_mps'), value_type=float),
                               'max_yaw_rate_rad_s': ParameterValue(LaunchConfiguration('max_yaw_rate_rad_s'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             parameters=[sim], output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',os.path.join(bringup,'config','competition_gazebo.rviz')]),
    ])
