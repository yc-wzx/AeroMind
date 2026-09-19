import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    return LaunchDescription([
        Node(
            package='uav_planning',
            executable='planar_cloud_projector',
            name='planar_cloud_projector',
            output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'planar_cloud_projector.yaml')],
        ),
        Node(
            package='uav_planning',
            executable='thin_odom_adapter',
            name='thin_odom_adapter',
            output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'thin_odom_adapter.yaml')],
        ),
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[os.path.join(
                bringup_share, 'config', 'ego_acl_offline.yaml')],
            remappings=[
                ('odom_world', '/ground/odometry'),
                ('grid_map/odom', '/ground/odometry'),
                ('grid_map/cloud', '/cloud_registered_2d'),
            ],
        ),
        Node(
            package='uav_planning',
            executable='ego_goal_adapter.py',
            name='ego_goal_adapter',
            output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'ego_goal_adapter.yaml')],
        ),
    ])
