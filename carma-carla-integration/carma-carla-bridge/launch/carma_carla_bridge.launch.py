from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    role_name = LaunchConfiguration('role_name')

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
            default_value='carma_1',
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
            name='autopilot',
            default_value='false',
            description='Determines if the spawned vehicle should use CARLA autopilot'
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

        # driver status params
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('controller_enabled', default_value='true'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('gnss_enabled', default_value='true'),
        DeclareLaunchArgument('driver_status_pub_rate', default_value='30'),

        # robot status params
        DeclareLaunchArgument('robot_status_pub_rate', default_value='30'),

        # route and plugins params
        DeclareLaunchArgument('selected_route', default_value=''),
        DeclareLaunchArgument('selected_plugins', default_value='/guidance/plugins/route_following_plugin,/guidance/plugins/inlanecruising_plugin,/guidance/plugins/stop_and_wait_plugin,/guidance/plugins/pure_pursuit_wrapper'),
        DeclareLaunchArgument('start_delay_in_seconds', default_value='10.0'),

        # ackermann control params
        DeclareLaunchArgument('init_speed', default_value='5.0'),
        DeclareLaunchArgument('init_acceleration', default_value='1.0'),
        DeclareLaunchArgument('init_steering_angle', default_value='0.0'),
        DeclareLaunchArgument('init_jerk', default_value='0.0'),
        DeclareLaunchArgument('max_steering_degree', default_value='70.0'),

        # enable sensor external
        DeclareLaunchArgument('enable_sensor_objects', default_value='false'),
        DeclareLaunchArgument('sensor_object_pub_rate', default_value='10'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('sensor_id', default_value='1'),
        DeclareLaunchArgument('detection_cycle_delay_seconds', default_value='0.1'),

        # ackermann speed/accel params
        DeclareLaunchArgument('speed_Kp', default_value='0.05'),
        DeclareLaunchArgument('speed_Ki', default_value='0.0'),
        DeclareLaunchArgument('speed_Kd', default_value='0.5'),
        DeclareLaunchArgument('accel_Kp', default_value='0.05'),
        DeclareLaunchArgument('accel_Ki', default_value='0.0'),
        DeclareLaunchArgument('accel_Kd', default_value='0.5'),

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
                'autopilot': LaunchConfiguration('autopilot'),
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
        ),


         ##################
        ## TF remapping ##
        ##################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '10']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_mobility',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'mobility', '10']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[role_name, '_to_baselink'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name], 'base_link', '10']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[role_name, 'gnss_to_gps'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name, '/gnss/gnss1'], 'gps', '10']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[role_name, 'lidar_to_velodyne'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name, '/lidar/lidar'], 'velodyne', '10']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[role_name, 'camerafront_to_camera'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name, '/camera/rgb/front'], 'camera', '10']
        ),

        
        #############################################
        ## topic remapping + data type conversions ##
        #############################################

        # localization #
        # Extract the pose, twist from carla odometry.
        Node(
            package='carma_carla_bridge',
            executable='carla_to_carma_localization',
            name='carla_to_carma_localization',
            output='screen',
            parameters=[{'role_name': role_name}]
        ),

        # external objects #
        # Extract the external objects from carla ObjectArray
        GroupAction([
            Node(
                package='carma_carla_bridge',
                executable='carla_to_carma_sensor_external_objects',
                name='carla_to_carma_sensor_external_objects',
                output='screen',
                parameters=[
                    {'role_name': role_name},
                    {'sensor_object_pub_rate': LaunchConfiguration('robot_status_pub_rate')},
                    {'host': LaunchConfiguration('host')},
                    {'port': LaunchConfiguration('port')},
                    {'sensor_id': LaunchConfiguration('sensor_id')},
                    {'detection_cycle_delay_seconds': LaunchConfiguration('detection_cycle_delay_seconds')}
                ]
            )
        ], condition=IfCondition(LaunchConfiguration('enable_sensor_objects'))),
        GroupAction([
            Node(
                package='carma_carla_bridge',
                executable='carla_to_carma_external_objects',
                name='carla_to_carma_external_objects',
                output='screen',
                parameters=[{'role_name': role_name}]
            )
        ], condition=UnlessCondition(LaunchConfiguration('enable_sensor_objects'))),

        # convert the vehicle status from carla to carma
        Node(
            package='carma_carla_bridge',
            executable='carla_to_carma_vehicle_status',
            name='carla_to_carma_vehicle_status',
            output='screen',
            parameters=[
                {'role_name': role_name},
                {'max_steering_degree': LaunchConfiguration('max_steering_degree')}
            ]
        ),
        Node(
            package='carma_carla_bridge',
            executable='carma_carla_robot_status',
            name='carla_to_carma_robot_status',
            output='screen',
            parameters=[{'robot_status_pub_rate': LaunchConfiguration('robot_status_pub_rate')}]
        ),
        Node(
            package='carma_carla_bridge',
            executable='carma_carla_driver_status',
            name='carla_to_carma_driver_status',
            output='screen',
            parameters=[
                {'driver_status_pub_rate': LaunchConfiguration('driver_status_pub_rate')},
                {'lidar_enabled': LaunchConfiguration('lidar_enabled')},
                {'controller_enabled': LaunchConfiguration('controller_enabled')},
                {'camera_enabled': LaunchConfiguration('camera_enabled')},
                {'gnss_enabled': LaunchConfiguration('gnss_enabled')}
            ]
        ),

        # route #
        # Set the vehicle route after localization.
        Node(
            package='carma_carla_bridge',
            executable='carma_carla_route',
            name='carma_carla_route',
            output='screen',
            parameters=[{'selected_route': LaunchConfiguration('selected_route')}]
        ),

        # plugins #
        # Activate all registered plugins.
        Node(
            package='carma_carla_bridge',
            executable='carma_carla_plugins',
            name='carma_carla_plugins',
            output='screen',
            parameters=[{'selected_plugins': LaunchConfiguration('selected_plugins')}]
        ),

        # guidance #
        # Set guidance to active once the plugins have been activated and the route has been selected.
        Node(
            package='carma_carla_bridge',
            executable='carma_carla_guidance',
            name='carma_carla_guidance',
            output='screen',
            parameters=[
                {'selected_route': LaunchConfiguration('selected_route')},
                {'selected_plugins': LaunchConfiguration('selected_plugins')},
                {'start_delay_in_seconds': LaunchConfiguration('start_delay_in_seconds')}
            ]
        ),

    ])