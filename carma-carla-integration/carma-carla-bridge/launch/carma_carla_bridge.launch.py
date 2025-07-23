from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('vehicle_filter', default_value='vehicle.toyota.prius'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument('spawn_point', default_value=''),

        DeclareLaunchArgument('speed_Kp', default_value='0.05'),
        DeclareLaunchArgument('speed_Ki', default_value='0.0'),
        DeclareLaunchArgument('speed_Kd', default_value='0.5'),

        DeclareLaunchArgument('accel_Kp', default_value='0.05'),
        DeclareLaunchArgument('accel_Ki', default_value='0.0'),
        DeclareLaunchArgument('accel_Kd', default_value='0.05'),

        DeclareLaunchArgument('synchronous_mode', default_value='false'),
        DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='false'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),
    ])