import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            'carla_host',
            default_value='localhost',
            description='Hostname of the CARLA server.'
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='TCP port of the CARLA server.'
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            'ego_vehicle_role_name',
            default_value='hero',
            description='Role name of the ego vehicle in CARLA.'
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            'carla_timeout',
            default_value='10.0',
            description='Timeout for CARLA client connection.'
        )
    )

    # Get launch config values
    carla_host = LaunchConfiguration('carla_host')
    carla_port = LaunchConfiguration('carla_port')
    ego_vehicle_role_name = LaunchConfiguration('ego_vehicle_role_name')
    carla_timeout = LaunchConfiguration('carla_timeout')

    # nodes
    vehicle_info_publisher_node = Node(
        package='carla_ros2_bridge',
        executable='vehicle_info_publisher_node',
        name='vehicle_info_publisher', # Node name in the ROS graph
        output='screen',
        parameters=[{
            'carla_host': carla_host,
            'carla_port': carla_port,
            'ego_vehicle_role_name': ego_vehicle_role_name,
            'update_frequency': 0.0, # Only publishes once for static info
            'carla_timeout': carla_timeout
        }]
    )

    odometry_publisher_node = Node(
        package='carla_ros2_bridge',
        executable='odometry_publisher_node',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'carla_host': carla_host,
            'carla_port': carla_port,
            'ego_vehicle_role_name': ego_vehicle_role_name,
            'update_frequency': 20.0,
            'carla_timeout': carla_timeout,
            'world_frame_id': 'map',
            'child_frame_id_prefix': '' # Results in child_frame_id being 'hero'
        }]
    )

    vehicle_status_publisher_node = Node(
        package='carla_ros2_bridge',
        executable='vehicle_status_publisher_node',
        name='vehicle_status_publisher',
        output='screen',
        parameters=[{
            'carla_host': carla_host,
            'carla_port': carla_port,
            'ego_vehicle_role_name': ego_vehicle_role_name,
            'update_frequency': 20.0,
            'carla_timeout': carla_timeout
        }]
    )

    objects_publisher_node = Node(
        package='carla_ros2_bridge',
        executable='objects_publisher_node',
        name='objects_publisher',
        output='screen',
        parameters=[{
            'carla_host': carla_host,
            'carla_port': carla_port,
            'ego_vehicle_role_name': ego_vehicle_role_name, # Used for topic namespacing and exclusion
            'update_frequency': 10.0,
            'carla_timeout': carla_timeout,
            'world_frame_id': 'map'
        }]
    )

    return LaunchDescription(declared_args + [
        vehicle_info_publisher_node,
        odometry_publisher_node,
        vehicle_status_publisher_node,
        objects_publisher_node
        # Add other nodes here if needed... Currently bridge only houses these four
    ])