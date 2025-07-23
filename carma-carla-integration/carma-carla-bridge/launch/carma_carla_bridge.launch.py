from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('timeout', default_value='10.0'),
        DeclareLaunchArgument('passive', default_value='False'),
        DeclareLaunchArgument('synchronous_mode', default_value='True'),
        DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='false'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),
        DeclareLaunchArgument('town', default_value='Town04'),
        DeclareLaunchArgument('role_name', default_value='carma_1'),
        DeclareLaunchArgument('vehicle_filter', default_value='vehicle.toyota.prius'),
        DeclareLaunchArgument('spawn_point', default_value='15.4,-90.1,0,0,0,90'),
        DeclareLaunchArgument('launch_spawn_vehicle', default_value='true'),
        DeclareLaunchArgument('launch_ackermann_control', default_value='true'),
        DeclareLaunchArgument('hero_config_path', default_value=PathJoinSubstitution([
            FindPackageShare('carla_ros2_bridge'), 'configs', 'carma_hero_stack.json'])),
        DeclareLaunchArgument('speed_Kp', default_value='0.4'),
        DeclareLaunchArgument('speed_Ki', default_value='0.03'),
        DeclareLaunchArgument('speed_Kd', default_value='0.0'),
        DeclareLaunchArgument('accel_Kp', default_value='0.05'),
        DeclareLaunchArgument('accel_Ki', default_value='0.0'),
        DeclareLaunchArgument('accel_Kd', default_value='0.05'),
    ])