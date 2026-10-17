from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    here=Path(__file__).resolve().parent
    return LaunchDescription([Node(package='spark_fast_lio',executable='spark_lio_mapping',name='mid360_real_lio',
        namespace='/hardware/mid360/lio',output='screen',
        parameters=[str(here.parent/'mid360_lio/lio_mid360_static.yaml')],
        remappings=[('lidar','/hardware/mid360/points'),('imu','/hardware/mid360/imu_si')])])
