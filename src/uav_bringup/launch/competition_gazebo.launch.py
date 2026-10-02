"""Gazebo field, holonomic robot, lidar, EGO navigation and RViz together."""
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
    world = os.path.join(bringup, 'worlds', 'competition_2025.sdf')
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '2', world],
                       output='log', additional_env={'QT_QPA_PLATFORM': 'xcb',
                           'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '4'})
    gui = ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '2', '--gui-config',
                 os.path.join(bringup, 'config', 'competition_gazebo_gui.config')],
            condition=IfCondition(LaunchConfiguration('gui')),
            output='log', additional_env={'QT_QPA_PLATFORM': 'xcb'})
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument('goal_delay_sec', default_value='10.0'),
        server, gui,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo window closed'))])),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gazebo_bridge', output='screen', arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                 '/gazebo/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                 '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                 '/model/omni_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
             ]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_identity', parameters=[sim], arguments=[
                 '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='uav_planning', executable='gazebo_navigation_interface.py',
             name='gazebo_navigation_interface', output='screen', parameters=[sim, {
                 'field_config': os.path.join(planning, 'config', 'competition_field_2025.json'),
                 'imperfect_sensors': False,
                 'auto_goal': ParameterValue(LaunchConfiguration('auto_goal'), value_type=bool),
                 'goal_delay_sec': ParameterValue(LaunchConfiguration('goal_delay_sec'), value_type=float),
             }]),
        Node(package='ego_planner', executable='ego_planner_node',
             name='ego_planner_node', output='log',
             parameters=[os.path.join(bringup, 'config', 'ego_competition_gazebo.yaml'), sim],
             remappings=[('odom_world','/ground/odometry'),
                         ('grid_map/odom','/ground/odometry'),
                         ('grid_map/cloud','/cloud_registered_2d')]),
        Node(package='uav_planning', executable='ego_goal_adapter.py',
             name='ego_goal_adapter', output='screen',
             parameters=[os.path.join(planning,'config','ego_goal_adapter.yaml'), sim,
                         {'input_topic': '/navigation/segment_goal'}]),
        Node(package='uav_planning', executable='ego_trajectory_executor.py',
             name='ego_trajectory_executor', output='screen',
             parameters=[os.path.join(planning,'config','ego_trajectory_executor.yaml'),
                         sim, {'enable_cmd_vel_output':True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             parameters=[sim], output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',os.path.join(bringup,'config','competition_gazebo.rviz')]),
    ])
