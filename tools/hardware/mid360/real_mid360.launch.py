"""Independent sensor/LIO-only entry. No planner, motor bridge, goal or cmd_vel."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent

def authorize_lio(context):
    if LaunchConfiguration('lio').perform(context).lower() not in ['true','1']:return []
    import sys
    sys.path.insert(0,str(ROOT))
    from preflight import verify_capture
    path=LaunchConfiguration('capture_summary').perform(context)
    if not path:raise RuntimeError('LIO blocked: first collect/review a passing real raw capture; capture_summary is required')
    verify_capture(path)
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('lio',default_value='false'),
        DeclareLaunchArgument('capture_summary',default_value=''),
        OpaqueFunction(function=authorize_lio),
        Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',
             name='mid360_real_driver',output='screen',parameters=[{
                 'use_sim_time':False,'xfer_format':0,'multi_topic':0,'data_src':0,
                 'publish_freq':10.,'output_data_type':0,'frame_id':'livox_frame',
                 'lvx_file_path':'','cmdline_input_bd_code':'',
                 'user_config_path':str(ROOT/'config/MID360_173.json')}],
             remappings=[('/livox/lidar','/hardware/mid360/points'),
                         ('/livox/imu','/hardware/mid360/imu_raw')]),
        ExecuteProcess(cmd=['python3','-B',str(ROOT/'normalize_imu.py')],output='screen'),
        Node(package='spark_fast_lio',executable='spark_lio_mapping',
             name='mid360_real_lio',namespace='/hardware/mid360/lio',
             condition=IfCondition(LaunchConfiguration('lio')),output='screen',
             parameters=[str(ROOT/'config/lio_mid360.yaml')],
             remappings=[('lidar','/hardware/mid360/points'),('imu','/hardware/mid360/imu_si')]),
        Node(package='rviz2',executable='rviz2',name='mid360_raw_rviz',
             condition=IfCondition(LaunchConfiguration('rviz')),output='log',
             parameters=[{'use_sim_time':False}],arguments=['-d',str(ROOT/'config/mid360_raw.rviz')]),
    ])
