import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches both the CARLA Bridge and the Ackermann Control nodes.
    """
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('timeout', default_value='10.0'),
        DeclareLaunchArgument('synchronous_mode', default_value='false'),
        DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='false'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),
        DeclareLaunchArgument('role_name', default_value='hero'),

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
            ]
        )
    ])