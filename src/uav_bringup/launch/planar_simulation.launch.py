"""Closed-loop software simulation for AeroMind planar navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    rviz = LaunchConfiguration('rviz')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_identity', output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),
        Node(
            package='uav_planning', executable='planar_navigation_simulator.py',
            name='planar_navigation_simulator', output='screen',
            parameters=[{'goal_x': goal_x, 'goal_y': goal_y, 'goal_yaw': goal_yaw}],
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
                {'enable_cmd_vel_output': True},
            ],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'ground_final.rviz')],
        ),
    ])
