from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    return LaunchDescription([
        ###################
        ## Configuration ##
        ###################
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
    ])