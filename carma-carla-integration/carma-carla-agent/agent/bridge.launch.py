from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # common params
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument('wheelbase', default_value='2.7'),

        # driver status params
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('controller_enabled', default_value='true'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('gnss_enabled', default_value='true'),
        DeclareLaunchArgument('driver_status_pub_rate', default_value='10'),

        # robot status params
        DeclareLaunchArgument('robot_status_pub_rate', default_value='10'),

        # route and plugins params
        DeclareLaunchArgument('selected_route', default_value=''),
        DeclareLaunchArgument('selected_plugins', default_value=''),
        DeclareLaunchArgument('start_delay_in_seconds', default_value='0'),

        # ackermann control params
        DeclareLaunchArgument('init_speed', default_value='5'),
        DeclareLaunchArgument('init_acceleration', default_value='1'),
        DeclareLaunchArgument('init_steering_angle', default_value='0'),
        DeclareLaunchArgument('init_jerk', default_value='0'),
        DeclareLaunchArgument('max_steering_degree', default_value='70'),

        # enable sensor external
        DeclareLaunchArgument('enable_sensor_objects', default_value='false'),
        DeclareLaunchArgument('sensor_object_pub_rate', default_value='10'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('sensor_id', default_value='1'),
        DeclareLaunchArgument('detection_cycle_delay_seconds', default_value='0.1'),

        
    ])