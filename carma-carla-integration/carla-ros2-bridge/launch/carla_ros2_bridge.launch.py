import launch
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches both the CARLA Bridge and the Ackermann Control nodes.
    """
    # Main bridge node that publishes sensor data
    carla_bridge_node = Node(
        package='carla_ros2_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            # Parameters are set with their correct Python data types
            {'use_sim_time': True},
            {'host': 'localhost'},
            {'port': 2000},
            {'timeout': 10.0},  # This is now a float, not a string
            {'synchronous_mode': True},
            {'fixed_delta_seconds': 0.05},
            {'ego_vehicle_role_name': 'hero'}
        ]
    )

    # Ackermann control node that translates CARMA commands
    ackermann_control_node = Node(
        package='carla_ros2_bridge',
        executable='ackermann_control',
        name='ackermann_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            # This node also needs the role_name to find the right topics
            {'role_name': 'hero'}
        ]
    )

    return launch.LaunchDescription([
        carla_bridge_node,
        ackermann_control_node
    ])