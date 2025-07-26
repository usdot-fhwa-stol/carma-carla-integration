from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='timeout',
            default_value='10.0',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='false',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        DeclareLaunchArgument(
            name='role_name',
            default_value='hero',
            description='Role name to identify ego vehicle, should match role_name in config at hero_config_path'
        ),
        DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*',
            description='Selects which vehicles are availiable in CARLA for spawn'
        ),
        DeclareLaunchArgument(
            name='spawn_point',
            default_value='None',
            description='Spawn point to be used for vehicle spawn in CARLA'
        ),
        DeclareLaunchArgument(
            name='launch_spawn_vehicle',
            default_value='true',
            description='Determines if spawn_hero_vehicle script is launched alongside carla-ros2-bridge node'
        ),
        DeclareLaunchArgument(
            name='launch_ackermann_control',
            default_value='false',
            description='Determines if ackermann control node is launched alongside carla-ros2-bridge node'
        ),
        DeclareLaunchArgument(
            name='hero_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('carla_ros2_bridge'),
                'configs',
                'stack.json'
            ]),
            description='Path to the hero vehicle JSON config'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('carla_ros2_bridge'),
                'launch',
                'carla_ros2_bridge.launch.py'
            ])),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'timeout': LaunchConfiguration('timeout'),
                'passive': LaunchConfiguration('passive'),
                'synchronous_mode': LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds'),
                'town': LaunchConfiguration('town'),
                'role_name': LaunchConfiguration('role_name'),
                'vehicle_filter': LaunchConfiguration('vehicle_filter'),
                'spawn_point': LaunchConfiguration('spawn_point'),
                'launch_spawn_vehicle': LaunchConfiguration('launch_spawn_vehicle'),
                'launch_ackermann_control': LaunchConfiguration('launch_ackermann_control'),
                'hero_config_path': LaunchConfiguration('hero_config_path'),
            }.items() | {
                'speed_Kp': LaunchConfiguration('speed_Kp'),
                'speed_Ki': LaunchConfiguration('speed_Ki'),
                'speed_Kd': LaunchConfiguration('speed_Kd'),
                'accel_Kp': LaunchConfiguration('accel_Kp'),
                'accel_Ki': LaunchConfiguration('accel_Ki'),
                'accel_Kd': LaunchConfiguration('accel_Kd'),
            }.items()
        )

    ])