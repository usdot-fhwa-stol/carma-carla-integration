#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handles communication between CARLA and ROS (Ported to ROS 2)
"""
import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread, Lock, Event

from rosgraph_msgs.msg import Clock

# Import all of our ported classes
from .actor import Actor
from .ego_vehicle import EgoVehicle
from .vehicle import Vehicle
from .walker import Walker
from .odom_sensor import OdometrySensor
from .object_sensor import ObjectSensor

class CarlaRosBridge(Node):
    """
    The main CARLA ROS 2 Bridge node.
    """
    def __init__(self):
        super().__init__("carla_ros_bridge")
        self.get_logger().info("CARLA ROS 2 Bridge Starting...")

        # Get parameters
        self.params = self._get_ros_parameters()
        self.carla_world = None
        self.actors = {}  # Dictionary to hold all our actor handlers {id -> Actor}
        self.ego_vehicle = None # Special handle for the ego vehicle

        # Main loop thread
        self.shutdown = Event()
        self.update_thread = Thread(target=self._update_loop)

    def _get_ros_parameters(self):
        """
        Reads the ROS 2 parameters and returns them as a dictionary.
        """
        params = {}
        params['host'] = self.declare_parameter('host', 'localhost').get_parameter_value().string_value
        params['port'] = self.declare_parameter('port', 2000).get_parameter_value().integer_value
        params['timeout'] = self.declare_parameter('timeout', 2.0).get_parameter_value().double_value
        params['synchronous_mode'] = self.declare_parameter('synchronous_mode', True).get_parameter_value().bool_value
        params['fixed_delta_seconds'] = self.declare_parameter('fixed_delta_seconds', 0.05).get_parameter_value().double_value
        
        # Get the role_name for the ego vehicle
        # For simplicity, we'll handle one ego vehicle. The original bridge handles a list.
        params['ego_vehicle_role_name'] = self.declare_parameter(
            'ego_vehicle_role_name', 'hero').get_parameter_value().string_value

        return params

    def initialize_bridge(self, carla_client):
        """
        Connects to the CARLA world and populates the actor list.
        """
        self.carla_world = carla_client.get_world()
        
        # Set synchronous mode settings
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = self.params['synchronous_mode']
        settings.fixed_delta_seconds = self.params['fixed_delta_seconds']
        self.carla_world.apply_settings(settings)
        
        self.get_logger().info("Finding and creating actors...")
        
        # Find all actors in the world and create our handler objects
        for actor in self.carla_world.get_actors():
            if actor.attributes.get('role_name') == self.params['ego_vehicle_role_name']:
                # Found the Ego Vehicle
                self.ego_vehicle = EgoVehicle(
                    uid=actor.id, name=actor.attributes.get('role_name', 'ego'),
                    parent=None, node=self, carla_actor=actor,
                    # We are not handling the control callback for now
                    vehicle_control_applied_callback=lambda id: None
                )
                self.actors[actor.id] = self.ego_vehicle
            elif isinstance(actor, carla.Vehicle):
                # Found a regular vehicle
                self.actors[actor.id] = Vehicle(
                    uid=actor.id, name=f"vehicle_{actor.id}",
                    parent=None, node=self, carla_actor=actor
                )
            elif isinstance(actor, carla.Walker):
                # Found a walker/pedestrian
                self.actors[actor.id] = Walker(
                    uid=actor.id, name=f"walker_{actor.id}",
                    parent=None, node=self, carla_actor=actor
                )

        if not self.ego_vehicle:
            self.get_logger().warning(f"Ego vehicle with role name '{self.params['ego_vehicle_role_name']}' not found in the world!")
            return

        # Now "sensor" components that are attached to the ego vehicle
        self.odometry_sensor = OdometrySensor(parent_actor=self.ego_vehicle, node=self)
        self.object_sensor = ObjectSensor(parent_actor=self.ego_vehicle, node=self, actor_list=self.actors)
        
        # Start the main update loop
        self.get_logger().info("Initialization complete. Starting update loop.")
        self.update_thread.start()

    def _update_loop(self):
        """
        The main execution loop of the bridge.
        """
        if not self.params['synchronous_mode']:
            self.get_logger().error("This ported bridge currently only supports synchronous mode.")
            return

        while rclpy.ok() and not self.shutdown.is_set():
            # Tick the CARLA world
            frame_id = self.carla_world.tick()
            
            # Get the timestamp from the world snapshot
            world_snapshot = self.carla_world.get_snapshot()
            timestamp = self.get_clock().now().to_msg() # Use ROS time for consistency
            
            # Update all actor states
            for actor_handler in self.actors.values():
                actor_handler.update(timestamp)

            # Update our "pseudo-sensors"
            self.odometry_sensor.update()
            self.object_sensor.update()

    def destroy(self):
        """
        Cleanly shutdown the bridge
        """
        self.get_logger().info("Shutting down CARLA ROS 2 Bridge...")
        self.shutdown.set()
        self.update_thread.join()
        
        # Destroy all managed actors
        for actor in self.actors.values():
            actor.destroy()
        
        # Destroy sensor components
        if self.odometry_sensor:
            self.odometry_sensor.destroy()
        if self.object_sensor:
            self.object_sensor.destroy()

        super().destroy_node()

def main(args=None):
    """
    Main function for the CARLA ROS 2 Bridge.
    """
    rclpy.init(args=args)
    
    carla_bridge = None
    carla_client = None
    
    try:
        carla_bridge = CarlaRosBridge()
        params = carla_bridge.params
        
        carla_bridge.get_logger().info(f"Connecting to CARLA at {params['host']}:{params['port']}...")
        carla_client = carla.Client(host=params['host'], port=params['port'])
        carla_client.set_timeout(params['timeout'])
        
        # Initialize the bridge with the CARLA client
        carla_bridge.initialize_bridge(carla_client)
        
        # Use a MultiThreadedExecutor to allow the main update loop to run in its own thread
        executor = MultiThreadedExecutor()
        executor.add_node(carla_bridge)
        
        # Spin the executor
        executor.spin()

    except (RuntimeError, IOError) as e:
        if carla_bridge:
            carla_bridge.get_logger().fatal(f"Error while running bridge: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if carla_bridge:
            carla_bridge.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()