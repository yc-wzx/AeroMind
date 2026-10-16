"""Independent physical inverted mount simulation; original navigation unchanged."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def circular_launch_preflight(context):
    if not IfCondition(LaunchConfiguration('navigation')).evaluate(context):return []
    import sys,json
    from ament_index_python.packages import get_package_prefix
    scripts=Path(get_package_prefix('provincial_rule900_scratch_sim'))/'lib/provincial_rule900_scratch_sim'
    sys.path.insert(0,str(scripts))
    from runtime_support import load_existing
    load_existing()
    from circle_route_gate import route_preflight
    from provincial_safety_geometry import wall_box
    from grid_route import GridRoute
    planning=Path(get_package_share_directory('uav_planning'))
    bringup=Path(get_package_share_directory('uav_bringup'))
    field=json.loads((Path(get_package_share_directory('provincial_rule900_scratch_sim'))/'config/provincial_rule900_training.json').read_text())
    walls=field['collision_polygons']
    grid=GridRoute(Path(get_package_share_directory('provincial_rule900_scratch_sim'))/'config/provincial_rule900_training.pgm',clearance=.4)
    for start,goal in [((4.7,.5),(8.7,4.25)),((8.7,4.25),(4.7,.5))]:
        result=route_preflight(start,goal,grid,walls,field['reference_route'])
        if not result['pass']:
            raise RuntimeError(result['reason']+'; launch prevented before starting Gazebo: '+json.dumps(result))
    return []


def generate_launch_description():
    bringup = get_package_share_directory('uav_bringup')
    planning = get_package_share_directory('uav_planning')
    sim = {'use_sim_time': True}
    fixed_config = os.path.join(get_package_share_directory('provincial_rule900_scratch_sim'), 'config', 'radar_fixed_sim.yaml')
    profile_params = os.path.join(get_package_share_directory('provincial_rule900_scratch_sim'), 'lio', 'parameters.yaml')
    candidate = get_package_share_directory('provincial_rule900_scratch_sim')
    original_inverted = candidate
    world = os.path.join(candidate, 'worlds', 'provincial_rule900_training.sdf')
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '2', world],
                       output='log', additional_env={'QT_QPA_PLATFORM': 'xcb',
                           'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '4', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.path.join(candidate,'..','..','lib')})
    gui = ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '2', '--gui-config',
                 os.path.join(bringup, 'config', 'competition_gazebo_gui.config')],
            condition=IfCondition(LaunchConfiguration('gui')),
            output='log', additional_env={'QT_QPA_PLATFORM': 'xcb','QT_X11_NO_MITSHM':'1','LIBGL_ALWAYS_SOFTWARE':'0'})
    return LaunchDescription([
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('qualification_trial', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('max_planar_speed_mps', default_value='0.25'),
        DeclareLaunchArgument('max_planar_acc_mps2', default_value='0.35'),
        DeclareLaunchArgument('max_yaw_rate_rad_s', default_value='0.30'),
        OpaqueFunction(function=circular_launch_preflight),
        server, gui,
        Node(package='provincial_rule900_scratch_sim', executable='radar_host_command.py', parameters=[fixed_config,sim], output='screen'),
        Node(package='provincial_rule900_scratch_sim', executable='radar_sim_electrical.py', parameters=[sim], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gazebo_bridge', output='screen', arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                 '/gazebo/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                 '/gimbal/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                 '/simulation/gimbal/feedback_enabled@std_msgs/msg/Bool]ignition.msgs.Boolean',
                 '/gimbal/target@std_msgs/msg/Float64]ignition.msgs.Double',
                 '/simulation/scan_inverted@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                 '/simulation/lidar3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                 '/simulation/imu3d@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                 '/simulation/imu_kinematics@std_msgs/msg/String[ignition.msgs.StringMsg',
                 '/model/omni_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
             ]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_identity', parameters=[sim], arguments=[
                 '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='provincial_rule900_scratch_sim', executable='inverted_sim_format_adapter.py',parameters=[sim],output='screen'),
        Node(package='provincial_rule900_scratch_sim', executable='gimbal_lio_mapping.py', name='isolated_lio_mapping',
             parameters=[str(profile_params),sim], output='log',
             arguments=['--simulation-only','true'],
             remappings=[('lidar','/lio/lidar'),('imu','/lio/imu'),('odometry','/lio/odometry'),
                         ('cloud_registered','/lio/cloud_registered'),('path','/lio/path')]),
        Node(package='provincial_rule900_scratch_sim', executable='radar_fixed_odometry.py',parameters=[fixed_config,sim],output='screen'),
        Node(package='provincial_rule900_scratch_sim', executable='circle_navigation.py',
             condition=IfCondition(LaunchConfiguration('navigation')),
             remappings=[('/gazebo/odometry', '/localization/lio_navigation_odometry'),
                         ('/model/omni_robot/cmd_vel','/navigation/guarded_body_cmd_vel')],
             name='gazebo_navigation_interface', output='screen', parameters=[fixed_config,sim, {
                 'field_config': os.path.join(candidate, 'config', 'provincial_rule900_training.json'),
                 'occupancy_map_pgm': os.path.join(candidate, 'config', 'provincial_rule900_training.pgm'),
                 'grid_route_clearance': 0.40,
                 'dynamic_goal_clearance': 0.40,
                 'imperfect_sensors': False,
                 'provincial_reference_route_mode': True,
                 'provincial_rect_guard': True,
                 'circle_envelope_radius_m': 0.30,
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
        Node(package='provincial_rule900_scratch_sim', executable='ego_goal_adapter.py',
             condition=IfCondition(LaunchConfiguration('navigation')),
             name='ego_goal_adapter', output='screen',
             parameters=[os.path.join(planning,'config','ego_goal_adapter.yaml'), sim,
                         {'input_topic': '/navigation/segment_goal'}]),
        Node(package='provincial_rule900_scratch_sim', executable='ego_trajectory_executor.py',
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
             arguments=['-d',os.path.join(candidate,'config','competition_gazebo.rviz')]),
    ])
