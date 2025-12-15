#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
# Developed by Will Varner/Ryan Fleming @ UGA MSC Lab 2025
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
"""
Rosbridge class:

Class that handles communication between CARLA and ROS (Ported to ROS 2)
"""
import carla
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from threading import Thread, Lock, Event
from math import sin, cos, radians
import time

from rosgraph_msgs.msg import Clock
import tf2_ros
from geometry_msgs.msg import TransformStamped

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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timestamp_last_run = 0.0

        # Clock publisher to keep ROS time synced with CARLA time (critical for odometry)
        self.clock_publisher = self.create_publisher(Clock, 'clock', 10)

        # Main loop thread (only used if we are the simulation MASTER)
        self.shutdown = Event()
        self.update_thread = None
        self.on_tick_id = None

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
        # Added passive parameter
        params['passive'] = self.declare_parameter('passive', False).get_parameter_value().bool_value

        # Get the role_name for the ego vehicle
        params['ego_vehicle_role_name'] = self.declare_parameter(
            'ego_vehicle_role_name', 'hero').get_parameter_value().string_value

        return params

    def initialize_bridge(self, carla_client):
        """
        Connects to the CARLA world and waits for the ego vehicle before initializing actors.
        """
        self.carla_world = carla_client.get_world()
        settings = self.carla_world.get_settings()

        # --- Synchronization / Passive Mode Logic ---
        if not self.params['passive']:
            # MASTER MODE: We control the simulation settings.
            # If we are NOT passive, we force the settings we want.
            if settings.synchronous_mode:
                # Workaround: disable sync mode briefly before applying new settings
                settings.synchronous_mode = False
                self.carla_world.apply_settings(settings)
            
            settings.synchronous_mode = self.params['synchronous_mode']
            settings.fixed_delta_seconds = self.params['fixed_delta_seconds']
            self.carla_world.apply_settings(settings)
            self.get_logger().info(f"[Bridge] Master Mode: Applied synchronous_mode={settings.synchronous_mode}, delta={settings.fixed_delta_seconds}")
        else:
            # PASSIVE MODE: We do NOT touch settings. We assume CDASim/XML-RPC handled it.
            self.get_logger().info("[Bridge] Passive Mode Enabled: Skipping setting application. Waiting for external tick.")

        # Traffic Manager config
        traffic_manager = carla_client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(self.params['synchronous_mode'])
        self.get_logger().info("[Bridge] Configured traffic manager")

        # --- Ego Vehicle Discovery ---
        self._wait_for_ego_vehicle()

        if not self.ego_vehicle:
            return # Error logged in helper

        # Populate remaining actors
        self._populate_actors()

        # Attach sensors
        self.odometry_sensor = OdometrySensor(parent_actor=self.ego_vehicle, node=self)
        self.object_sensor = ObjectSensor(parent_actor=self.ego_vehicle, node=self, actor_list=self.actors)
        
        # Publish static TF for sensors
        self._publish_static_transforms()

        # --- Start Execution Loop ---
        # Logic derived from legacy bridge:
        # If Synchronous AND Passive: Use on_tick callback (Listen)
        # If Synchronous AND NOT Passive: Use Thread loop (Drive)
        
        if self.params['synchronous_mode'] and not self.params['passive']:
            self.get_logger().info("[Bridge] Starting Active Update Thread (Driving Simulation)")
            self.update_thread = Thread(target=self._active_update_loop)
            self.update_thread.start()
        elif self.params['synchronous_mode'] and self.params['passive']:
            self.get_logger().info("[Bridge] Registering on_tick callback (Passive Listener)")
            self.on_tick_id = self.carla_world.on_tick(self._on_passive_tick)
        else:
            self.get_logger().warn("[Bridge] Asynchronous mode is not fully supported in this port yet.")

    def _wait_for_ego_vehicle(self):
        """Helper to block until ego vehicle is found"""
        timeout_seconds = 30
        poll_interval = 1.0
        waited = 0

        self.get_logger().info(f"[Bridge] Waiting for ego vehicle with role_name '{self.params['ego_vehicle_role_name']}'")

        while rclpy.ok() and waited < timeout_seconds:
            actors = self.carla_world.get_actors()
            for actor in actors:
                if hasattr(actor, 'attributes'):
                    role_name = actor.attributes.get('role_name', 'None')
                    if role_name == self.params['ego_vehicle_role_name']:
                        self.ego_vehicle = EgoVehicle(
                            uid=actor.id, name=actor.attributes.get('role_name', 'ego'),
                            parent=None, node=self, carla_actor=actor,
                            vehicle_control_applied_callback=lambda id: None
                        )
                        self.actors[actor.id] = self.ego_vehicle
                        self.get_logger().info(f"[Bridge] Found ego vehicle with role_name '{self.ego_vehicle.name}'")
                        return
            time.sleep(poll_interval)
            waited += poll_interval

        self.get_logger().error(f"[Bridge] Ego vehicle with role name '{self.params['ego_vehicle_role_name']}' not found!")

    def _populate_actors(self):
        """Helper to convert CARLA actors to bridge actors"""
        for actor in self.carla_world.get_actors():
            if actor.id == self.ego_vehicle.uid:
                continue
            elif isinstance(actor, carla.Vehicle):
                self.actors[actor.id] = Vehicle(
                    uid=actor.id, name=f"vehicle_{actor.id}",
                    parent=None, node=self, carla_actor=actor
                )
            elif isinstance(actor, carla.Walker):
                self.actors[actor.id] = Walker(
                    uid=actor.id, name=f"walker_{actor.id}",
                    parent=None, node=self, carla_actor=actor
                )

    def _active_update_loop(self):
        """
        MASTER MODE LOOP: We are responsible for ticking the world.
        """
        while rclpy.ok() and not self.shutdown.is_set():
            # 1. Drive the simulation
            frame_id = self.carla_world.tick()
            
            # 2. Get the resulting state
            snapshot = self.carla_world.get_snapshot()
            if snapshot:
                self._process_snapshot(snapshot)

    def _on_passive_tick(self, snapshot):
        """
        PASSIVE MODE CALLBACK: CDASim (or XML-RPC) ticked the world. We just react.
        """
        if not self.shutdown.is_set():
            # Ensure we process frames in order and don't duplicate
            if self.timestamp_last_run < snapshot.timestamp.elapsed_seconds:
                self.timestamp_last_run = snapshot.timestamp.elapsed_seconds
                self._process_snapshot(snapshot)

    def _process_snapshot(self, snapshot):
        frame_id = snapshot.frame
        carla_timestamp = snapshot.timestamp
        
        # 1. Construct the ROS Time object manually from the snapshot
        # This creates a "Source of Truth" that eliminates the race condition
        seconds = int(carla_timestamp.elapsed_seconds)
        nanoseconds = int((carla_timestamp.elapsed_seconds - seconds) * 1e9)
        current_ros_time = Time(seconds=seconds, nanoseconds=nanoseconds)
        
        # 2. Publish this time to /clock
        clock_msg = Clock()
        clock_msg.clock = current_ros_time.to_msg()
        self.clock_publisher.publish(clock_msg)

        # 3. Use the EXACT same time for headers
        # Do NOT use self.get_clock().now() here!
        ros_timestamp_msg = current_ros_time.to_msg() 

        self.get_logger().debug(
            f"Processing frame={frame_id} sim_time={carla_timestamp.elapsed_seconds:.3f}"
        )

        # 4. Broadcast Transforms
        if self.ego_vehicle:
            self._broadcast_ego_transform(ros_timestamp_msg)

        # 5. Update Actors
        for actor_handler in self.actors.values():
            actor_handler.update(ros_timestamp_msg)

        # 6. Update Sensors (CRITICAL: Pass the timestamp explicitly)
        # You must update your OdometrySensor.update() method to accept this argument!
        self.odometry_sensor.update(ros_timestamp_msg)
        self.object_sensor.update(ros_timestamp_msg)

    def update_clock(self, carla_timestamp):
        """
        Manually publish the /clock topic to sync ROS with CARLA simulation time.
        """
        msg = Clock()
        # Convert seconds to ROS Time (sec, nanosec)
        seconds = int(carla_timestamp.elapsed_seconds)
        nanoseconds = int((carla_timestamp.elapsed_seconds - seconds) * 1e9)
        msg.clock.sec = seconds
        msg.clock.nanosec = nanoseconds
        self.clock_publisher.publish(msg)

    def _broadcast_ego_transform(self, ros_timestamp):
        """Helper to broadcast ego TF"""
        carla_transform = self.ego_vehicle.carla_actor.get_transform()
        loc = carla_transform.location
        rot = carla_transform.rotation
        
        t = TransformStamped()
        t.header.stamp = ros_timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = self.params['ego_vehicle_role_name']

        # Coordinate conversion (Left-handed CARLA -> Right-handed ROS)
        t.transform.translation.x = loc.x
        t.transform.translation.y = -loc.y 
        t.transform.translation.z = loc.z

        roll = radians(rot.roll)
        pitch = -radians(rot.pitch) 
        yaw = -radians(rot.yaw)     

        cy = cos(yaw * 0.5); sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5); sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5); sr = sin(roll * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self.tf_broadcaster.sendTransform(t)

    def _publish_static_transforms(self):
        """Publishes static TFs for attached sensors once at startup"""
        static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        transforms_to_publish = []
        for actor in self.carla_world.get_actors():
            if actor.parent and actor.parent.id == self.ego_vehicle.uid:
                sensor_transform = actor.get_transform()
                loc = sensor_transform.location
                rot = sensor_transform.rotation
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.params['ego_vehicle_role_name']
                child_frame_id = actor.attributes.get('role_name', f"sensor_{actor.id}")
                t.child_frame_id = child_frame_id
                
                t.transform.translation.x = loc.x
                t.transform.translation.y = -loc.y
                t.transform.translation.z = loc.z
                
                roll = radians(rot.roll)
                pitch = -radians(rot.pitch)
                yaw = -radians(rot.yaw)
                cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
                cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
                cr = cos(roll * 0.5); sr = sin(roll * 0.5);
                t.transform.rotation.w = cr * cp * cy + sr * sp * sy
                t.transform.rotation.x = sr * cp * cy - cr * sp * sy
                t.transform.rotation.y = cr * sp * cy + sr * cp * sy
                t.transform.rotation.z = cr * cp * sy - sr * sp * cy
                transforms_to_publish.append(t)
                self.get_logger().info(f"Prepared static TF for sensor: {child_frame_id}")
        static_tf_broadcaster.sendTransform(transforms_to_publish)

    def destroy(self):
        """
        Cleanly shutdown the bridge
        """
        self.get_logger().info("Shutting down CARLA ROS 2 Bridge...")
        self.shutdown.set()
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join()
        
        # Cleanup Passive Callback
        if self.on_tick_id:
            try:
                self.carla_world.remove_on_tick(self.on_tick_id)
            except:
                pass

        for actor in self.actors.values():
            actor.destroy()

        if hasattr(self, 'odometry_sensor') and self.odometry_sensor:
            self.odometry_sensor.destroy()
        if hasattr(self, 'object_sensor') and self.object_sensor:
            self.object_sensor.destroy()

        super().destroy_node()

def main(args=None):
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
