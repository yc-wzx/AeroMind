"""Main AeroMind launch for a planar holonomic robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    map_path = LaunchConfiguration('map_path')
    use_relocalization = LaunchConfiguration('use_relocalization')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value='/home/rmnav/AeroMind/maps/demo/lab_map.pcd'),
        DeclareLaunchArgument('use_relocalization', default_value='true'),
        DeclareLaunchArgument(
            'enable_cmd_vel', default_value='false',
            description='Explicit output gate for holonomic /cmd_vel'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'live_mid360_lio.launch.py')),
        ),
        Node(
            package='uav_planning', executable='thin_odom_adapter',
            name='planar_odom_adapter', output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'thin_odom_adapter.yaml')],
        ),
        Node(
            package='uav_planning', executable='planar_cloud_projector',
            name='planar_cloud_projector', output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'planar_cloud_projector.yaml')],
        ),
        Node(
            package='ego_planner', executable='ego_planner_node',
            name='ego_planner_node', output='screen',
            parameters=[os.path.join(
                bringup_share, 'config', 'ego_acl_offline.yaml')],
            remappings=[
                ('odom_world', '/ground/odometry'),
                ('grid_map/odom', '/ground/odometry'),
                ('grid_map/cloud', '/cloud_registered_2d'),
            ],
        ),
        Node(
            package='uav_planning', executable='ego_goal_adapter.py',
            name='ego_goal_adapter', output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'ego_goal_adapter.yaml')],
        ),
        Node(
            package='uav_planning', executable='ego_trajectory_executor.py',
            name='ego_trajectory_executor', output='screen',
            parameters=[
                os.path.join(planning_share, 'config', 'ego_trajectory_executor.yaml'),
                {'enable_cmd_vel_output': enable_cmd_vel},
            ],
        ),
        Node(
            package='uav_planning', executable='kiss_relocalizer',
            name='kiss_relocalizer', output='screen',
            condition=IfCondition(use_relocalization),
            parameters=[
                os.path.join(planning_share, 'config', 'kiss_relocalizer.yaml'),
                {'map_path': map_path},
            ],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_identity', output='screen',
            condition=UnlessCondition(use_relocalization),
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'ground_final.rviz')],
        ),
    ])
