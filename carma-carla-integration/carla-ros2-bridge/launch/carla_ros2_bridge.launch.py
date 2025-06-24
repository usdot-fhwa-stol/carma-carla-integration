import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launches both the CARLA Bridge and conditionally the Ackermann Control node and vehicle spawn script.
    """
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(name='host', default_value='localhost'),
        DeclareLaunchArgument(name='port', default_value='2000'),
        DeclareLaunchArgument(name='timeout', default_value='10.0'),
        DeclareLaunchArgument(name='role_name', default_value='hero'),
        DeclareLaunchArgument(name='vehicle_filter', default_value='vehicle.*'),
        DeclareLaunchArgument(name='spawn_point', default_value='None'),
        DeclareLaunchArgument(name='town', default_value='Town01'),
        DeclareLaunchArgument(name='passive', default_value='False'),
        DeclareLaunchArgument(name='synchronous_mode', default_value='false'),
        DeclareLaunchArgument(name='synchronous_mode_wait_for_vehicle_control_command', default_value='false'),
        DeclareLaunchArgument(name='fixed_delta_seconds', default_value='0.05'),
        DeclareLaunchArgument(name='launch_spawn_hero_vehicle', default_value='true'),
        DeclareLaunchArgument(name='launch_ackermann_control', default_value='false'),
        DeclareLaunchArgument(
            name='hero_config_path',
            default_value=TextSubstitution(text=os.path.join(
                get_package_share_directory('carla_ros2_bridge'),
                'configs',
                'stack.json'
            )),
            description='Path to the hero vehicle JSON config'
        ),

        # Main CARLA bridge node
        Node(
            package='carla_ros2_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True},
                {'host': LaunchConfiguration('host')},
                {'port': LaunchConfiguration('port')},
                {'timeout': LaunchConfiguration('timeout')},
                {'synchronous_mode': LaunchConfiguration('synchronous_mode')},
                {'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')},
                {'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds')},
                {'ego_vehicle_role_name': LaunchConfiguration('role_name')},
            ]
        ),
        
        # Ackermann control node
        Node(
            package='carla_ros2_bridge',
            executable='ackermann_control',
            name='ackermann_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'role_name': LaunchConfiguration('role_name')},
                {'speed_Kp': 0.05},
                {'speed_Ki': 0.0},
                {'speed_Kd': 0.5},
                {'accel_Kp': 0.05},
                {'accel_Ki': 0.0},
                {'accel_Kd': 0.05},
            ],  
            condition=IfCondition(LaunchConfiguration('launch_ackermann_control')),
        ),

        # Vehicle spawn script
        Node(
            package='carla_ros2_bridge',
            executable='spawn_hero_vehicle',
            name='spawn_vehicle_node',
            output='screen',
            emulate_tty=True,
            arguments=[
                '--host', LaunchConfiguration('host'),
                '--port', LaunchConfiguration('port'),
                '--file', LaunchConfiguration('hero_config_path'),
            ],
            condition=IfCondition(LaunchConfiguration('launch_spawn_hero_vehicle')),
        ),
    ])