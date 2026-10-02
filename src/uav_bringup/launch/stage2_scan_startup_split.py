"""Test orchestration only: partition the existing launch without copying parameters."""
from pathlib import Path
import runpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def split_description(navigation_only):
    source=Path(get_package_share_directory('uav_bringup'))/'launch/provincial_stage2_scan_resume.launch.py'
    original=runpy.run_path(str(source))['generate_launch_description']()
    navigation=[a for a in original.entities if isinstance(a,Node) and
        a.node_executable=='stage2_localized_interface.py']
    if len(navigation)!=1:raise RuntimeError('Expected exactly one original localization node')
    nav=navigation[0]
    if navigation_only:
        return LaunchDescription([a for a in original.entities if isinstance(a,DeclareLaunchArgument) or a is nav])
    return LaunchDescription([a for a in original.entities if a is not nav])
