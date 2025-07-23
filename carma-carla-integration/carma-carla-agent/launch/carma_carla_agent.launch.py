from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

def generate_launch_description():
    return LaunchDescription([
        ###################
        ## Configuration ##
        ###################
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('agent', default_value='agent'),
        #  connecting default info 
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town04'),

        # set synchronous_mode to 'true' when carma-carla-integration not running with co-simulation tool
        DeclareLaunchArgument('synchronous_mode', default_value='false'),
        DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='false'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),

        # use comma separated format "x,y,z,roll,pìtch,yaw"
        DeclareLaunchArgument('spawn_point', default_value='15.4,-90.1,0,0,0,90'),

        # vehicle info
        DeclareLaunchArgument('role_name', default_value='carma_1'),
        DeclareLaunchArgument('vehicle_model', default_value='vehicle.toyota.prius'),
        DeclareLaunchArgument('vehicle_length', default_value='5.00'),
        DeclareLaunchArgument('vehicle_width', default_value='3.00'),
        DeclareLaunchArgument('vehicle_wheelbase', default_value='2.79'),

        # vehicle speed PID
        DeclareLaunchArgument('speed_Kp', default_value='0.4'),
        DeclareLaunchArgument('speed_Ki', default_value='0.03'),
        DeclareLaunchArgument('speed_Kd', default_value='0.0'),

        # vehicle acceleration PID
        DeclareLaunchArgument('accel_Kp', default_value='0.05'),
        DeclareLaunchArgument('accel_Ki', default_value='0.0'),
        DeclareLaunchArgument('accel_Kd', default_value='0.05'),

        # Initial arguments for integration scripts
        DeclareLaunchArgument('init_speed', default_value='5'),
        DeclareLaunchArgument('init_acceleration', default_value='1'),
        DeclareLaunchArgument('init_steering_angle', default_value='0'),
        DeclareLaunchArgument('init_jerk', default_value='0'),
        DeclareLaunchArgument('max_steering_degree', default_value='70'),

        DeclareLaunchArgument('use_ground_truth_localization', default_value='false'),
        DeclareLaunchArgument('use_ground_truth_detections', default_value='false'),

        # driver status params
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('controller_enabled', default_value='true'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('gnss_enabled', default_value='true'),
        DeclareLaunchArgument('driver_status_pub_rate', default_value='10'),

        # robot status params 
        DeclareLaunchArgument('robot_status_pub_rate', default_value='10'),

        # route and plugins params
        DeclareLaunchArgument('selected_route', default_value='Release_test_case_1'),
        DeclareLaunchArgument('selected_plugins', default_value="['/guidance/plugins/route_following_plugin','/guidance/plugins/inlanecruising_plugin','/guidance/plugins/stop_and_wait_plugin','/guidance/plugins/pure_pursuit_wrapper']"),
        DeclareLaunchArgument('start_delay_in_seconds', default_value='15.0'),

        # enable sensor external object
        DeclareLaunchArgument('enable_sensor_objects', default_value='false'),
        DeclareLaunchArgument('sensor_object_pub_rate', default_value='10'),
        DeclareLaunchArgument('sensor_id', default_value='1'),
        DeclareLaunchArgument('detection_cycle_delay_seconds', default_value='0.1'),

        ##########################
        ##  CARLA CARMA bridge  ##
        ##########################
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('carma_carla_bridge'),
                    'launch',
                    'carma_carla_bridge.launch.py'
                ])
            ]),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'vehicle_filter': LaunchConfiguration('vehicle_model'),
                'role_name': LaunchConfiguration('role_name'),
                'spawn_point': LaunchConfiguration('spawn_point'),
                'speed_Kp': LaunchConfiguration('speed_Kp'),
                'speed_Ki': LaunchConfiguration('speed_Ki'),
                'speed_Kd': LaunchConfiguration('speed_Kd'),
                'accel_Kp': LaunchConfiguration('accel_Kp'),
                'accel_Ki': LaunchConfiguration('accel_Ki'),
                'accel_Kd': LaunchConfiguration('accel_Kd'),
                'synchronous_mode': LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds'),
            }.items()
        ),

        ##################
        ## Agent bridge ##
        ##################
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('carma_carla_agent'),
                    LaunchConfiguration('agent'),
                    'bridge.launch.py'
                ])
            ]),
            launch_arguments={
                'role_name': LaunchConfiguration('role_name'),
                'wheelbase': LaunchConfiguration('vehicle_wheelbase'),
                'init_speed': LaunchConfiguration('init_speed'),
                'init_acceleration': LaunchConfiguration('init_acceleration'),
                'init_steering_angle': LaunchConfiguration('init_steering_angle'),
                'init_jerk': LaunchConfiguration('init_jerk'),
                'max_steering_degree': LaunchConfiguration('max_steering_degree'),
                'lidar_enabled': LaunchConfiguration('lidar_enabled'),
                'controller_enabled': LaunchConfiguration('controller_enabled'),
                'camera_enabled': LaunchConfiguration('camera_enabled'),
                'gnss_enabled': LaunchConfiguration('gnss_enabled'),
                'driver_status_pub_rate': LaunchConfiguration('driver_status_pub_rate'),
                'robot_status_pub_rate': LaunchConfiguration('robot_status_pub_rate'),
                'selected_route': LaunchConfiguration('selected_route'),
                'selected_plugins': LaunchConfiguration('selected_plugins'),
                'start_delay_in_seconds': LaunchConfiguration('start_delay_in_seconds'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'enable_sensor_objects': LaunchConfiguration('enable_sensor_objects'),
                'sensor_object_pub_rate': LaunchConfiguration('sensor_object_pub_rate'),
                'sensor_id': LaunchConfiguration('sensor_id'),
                'detection_cycle_delay_seconds': LaunchConfiguration('detection_cycle_delay_seconds'),
            }.items()
        )

    ])