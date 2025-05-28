#!/usr/bin/env python3
# Copyright 2025 Will Varner, UGA MSC Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS 2 Node to publish information about dynamic objects (vehicles, pedestrians)
in the CARLA world.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
# REMOVED: from rclpy.exceptions import ParameterNotSetException 

import carla
from itertools import chain # To combine iterators for different actor types

# Import standard ROS messages
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3 # For twist.angular if needed, accel.angular
from shape_msgs.msg import SolidPrimitive

# Import the message types for detected objects
from derived_object_msgs.msg import Object, ObjectArray

# Import your transformations utility
from . import carla_utils
from . import transforms # Assuming transforms.py is in the same Python package


class ObjectsPublisherNode(Node):
    """
    Publishes an array of detected dynamic objects from the CARLA simulation.
    """
    def __init__(self):
        super().__init__('objects_publisher_node')

        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('ego_vehicle_role_name', 'hero')
        self.declare_parameter('update_frequency', 10.0) # Hz
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('world_frame_id', 'map')


        # Get parameters directly - this is safe because defaults are provided
        self.host = self.get_parameter('carla_host').get_parameter_value().string_value
        self.port = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.role_name = self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value # Used for topic namespacing
        self.ego_vehicle_role_name_to_exclude = self.role_name # Assuming it's the same role_name for exclusion
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.carla_timeout = self.get_parameter('carla_timeout').get_parameter_value().double_value
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        
        self.client = None
        self.world = None
        self.ego_vehicle_id = -1 # To store the ID of the ego vehicle to exclude it
        self.synchronous_mode_settings = None

        # QoS for objects: Typically Reliable, Volatile.
        objects_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(
            ObjectArray,
            f'/carla/{self.role_name}/objects', 
            objects_qos_profile
        )

        if not self._connect_to_carla_and_setup_world():
            self.get_logger().error("CARLA connection/world setup failed. Node will not publish objects.")

        if self.world and self.update_frequency > 0:
            self.timer = self.create_timer(1.0 / self.update_frequency, self._publish_objects)
        else:
            self.get_logger().warn(f"Timer not started for objects. World: {self.world}, Update Freq: {self.update_frequency}")

        self.get_logger().info(f"Objects Publisher for reference role '{self.role_name}' initialized.")
        self.get_logger().info(f"Publishing to /carla/{self.role_name}/objects at {self.update_frequency} Hz.")

    def _connect_to_carla_and_setup_world(self) -> bool:
        """Establish connection and set up CARLA world for synchronous mode if needed."""
        self.get_logger().info(f"Attempting to connect to CARLA at {self.host}:{self.port}...")
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.carla_timeout)
            self.world = self.client.get_world()
            self.world.get_snapshot() 

            self.synchronous_mode_settings = self.world.get_settings()
            settings = self.world.get_settings()
            if self.update_frequency > 0:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 1.0 / self.update_frequency
            else:
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            self.get_logger().info(f"CARLA world synchronous_mode: {settings.synchronous_mode}, fixed_delta_seconds: {settings.fixed_delta_seconds}")
            
            traffic_manager = self.client.get_trafficmanager() # Add self.client.get_tm_port() if needed
            traffic_manager.set_synchronous_mode(True)
            
            self.get_logger().info("Successfully connected to CARLA server and configured world settings for Objects Publisher.")
            return True
        except RuntimeError as e:
            self.get_logger().error(f"Failed to connect to CARLA or setup world for Objects Publisher: {e}.")
            self.world = None; self.client = None
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error during CARLA connection/setup for Objects Publisher: {e}")
            self.world = None; self.client = None
            return False

    def _find_and_store_ego_vehicle_id(self):
        """Finds the ego vehicle and stores its ID to exclude it from the object list."""
        if not self.world: return
        if self.ego_vehicle_id != -1: return # Already found

        # Filter by role_name directly if possible, otherwise iterate.
        # CARLA's actor_list.filter() usually takes wildcards like 'vehicle.*'.
        # For specific role_name, iterating is often more reliable.
        for actor in self.world.get_actors():
             if actor.attributes.get('role_name') == self.ego_vehicle_role_name_to_exclude:
                self.ego_vehicle_id = actor.id
                self.get_logger().info(f"Ego vehicle '{self.ego_vehicle_role_name_to_exclude}' with ID {self.ego_vehicle_id} will be excluded from object list.")
                return # Found it
        self.get_logger().warn(f"Ego vehicle with role_name '{self.ego_vehicle_role_name_to_exclude}' not found to exclude.")


    def _publish_objects(self):
        """Fetches data about other vehicles and pedestrians and publishes them."""
        if not self.world:
            self.get_logger().warn("No CARLA world, skipping objects publish.")
            if not self._connect_to_carla_and_setup_world(): return
        
        if self.synchronous_mode_settings and self.synchronous_mode_settings.synchronous_mode:
            try:
                self.world.tick()
            except RuntimeError as e:
                self.get_logger().warn(f"RuntimeError during world.tick() for objects: {e}.")
                self.ego_vehicle_id = -1 
                return

        if self.ego_vehicle_id == -1: # Attempt to find it if not yet found
            self._find_and_store_ego_vehicle_id()

        object_array_msg = ObjectArray()
        object_array_msg.header.stamp = self.get_clock().now().to_msg()
        object_array_msg.header.frame_id = self.world_frame_id

        relevant_actors = chain(
            self.world.get_actors().filter('vehicle.*'),
            self.world.get_actors().filter('walker.pedestrian.*')
        )

        for actor in relevant_actors:
            if not actor.is_alive:
                continue
            if actor.id == self.ego_vehicle_id: 
                continue

            try:
                obj = Object()
                obj.header = object_array_msg.header 
                obj.id = actor.id 

                carla_transform = actor.get_transform()
                obj.pose = transforms.carla_transform_to_ros_pose(carla_transform)

                carla_linear_velocity = actor.get_velocity()
                carla_angular_velocity = actor.get_angular_velocity() 
                ros_twist = transforms.carla_velocity_to_ros_twist(
                    carla_linear_velocity, carla_angular_velocity)
                obj.twist = ros_twist

                carla_accel = actor.get_acceleration()
                ros_accel = transforms.carla_acceleration_to_ros_accel(carla_accel)
                obj.accel = ros_accel
                
                carla_bbox = actor.bounding_box
                obj.shape.type = SolidPrimitive.BOX
                obj.shape.dimensions = [ 
                    float(carla_bbox.extent.x * 2.0),
                    float(carla_bbox.extent.y * 2.0),
                    float(carla_bbox.extent.z * 2.0)
                ]

                obj.object_classified = True 
                if 'vehicle' in actor.type_id:
                    obj.classification = Object.CLASSIFICATION_CAR
                elif 'walker.pedestrian' in actor.type_id:
                    obj.classification = Object.CLASSIFICATION_PEDESTRIAN
                else:
                    obj.classification = Object.CLASSIFICATION_UNKNOWN
                
                object_array_msg.objects.append(obj)

            except RuntimeError as e:
                self.get_logger().debug(f"Could not process actor {actor.id} ({actor.type_id}): {e}. Might have been destroyed.")
            except Exception as e:
                self.get_logger().warn(f"Unexpected error processing actor {actor.id} ({actor.type_id}): {e}", exc_info=True)

        self.publisher_.publish(object_array_msg)
        self.get_logger().debug(f"Published {len(object_array_msg.objects)} objects.")


    def restore_world_settings(self):
        """Restores original CARLA world settings if they were changed."""
        if self.synchronous_mode_settings and self.world:
            self.get_logger().info("Attempting to restore original CARLA world settings (from objects publisher)...")
            try:
                self.world.apply_settings(self.synchronous_mode_settings)
                self.get_logger().info("Original CARLA world settings restored (from objects publisher).")
            except RuntimeError as e:
                self.get_logger().error(f"Failed to restore CARLA world settings (from objects publisher): {e}")

    def destroy_node(self):
        """Node cleanup."""
        self.get_logger().info("Shutting down Objects Publisher Node...")
        if self.timer and not self.timer.is_canceled():
            self.timer.cancel()
        self.restore_world_settings()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectsPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Keyboard interrupt received, shutting down objects publisher.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in ObjectsPublisherNode: {e}", exc_info=True)
        else:
            print(f"[FATAL] Unhandled exception during objects publisher node initialization: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()