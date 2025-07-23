from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    role_name = LaunchConfiguration('role_name')

    return LaunchDescription([
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

        ##################
        ## TF remapping ##
        ##################
        Node(
            package='tf',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '10']
        ),
        Node(
            package='tf',
            executable='static_transform_publisher',
            name='map_to_mobility',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'mobility', '10']
        ),
        Node(
            package='tf',
            executable='static_transform_publisher',
            name=[role_name, '_to_baselink'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name], 'base_link', '10']
        ),
        Node(
            package='tf',
            executable='static_transform_publisher',
            name=[role_name, 'gnss_to_gps'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name, '/gnss/gnss1'], 'gps', '10']
        ),
        Node(
            package='tf',
            executable='static_transform_publisher',
            name=[role_name, 'lidar_to_velodyne'],
            arguments=['0', '0', '0', '0', '0', '0', [role_name, '/lidar/lidar'], 'velodyne', '10']
        ),
        Node(
            package='tf',
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
                executable='carla_to_carma_external_objects',
                name='carla_to_carma_external_objects',
                output='screen',
                parameters=[{'role_name': role_name}]
            )
        ], condition=IfCondition(LaunchConfiguration('enable_sensor_objects'))),
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
        ], condition=UnlessCondition(LaunchConfiguration('enable_sensor_objects'))),

        # convert vehicle command to carla ackermann drive
        Node(
            package='carma_carla_bridge',
            executable='carma_to_carla_ackermann_cmd',
            name='carma_to_carla_ackermann_cmd',
            output='screen',
            parameters=[
                {'role_name': role_name},
                {'init_speed': LaunchConfiguration('init_speed')},
                {'init_acceleration': LaunchConfiguration('init_acceleration')},
                {'init_steering_angle': LaunchConfiguration('init_steering_angle')},
                {'init_jerk': LaunchConfiguration('init_jerk')}
            ]
        ),

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