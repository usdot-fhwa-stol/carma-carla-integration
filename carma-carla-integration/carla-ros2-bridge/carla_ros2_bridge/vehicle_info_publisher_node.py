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
ROS 2 Node to publish static vehicle information from CARLA.

This node connects to a CARLA server, finds a specified ego vehicle,
and publishes its static physical and descriptive information. The topic
is latched, and the node typically publishes information once.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import carla
import math
import traceback # For detailed exception logging

# Import the custom CARLA messages
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel

# Import standard ROS messages
from geometry_msgs.msg import Point, Vector3
from shape_msgs.msg import SolidPrimitive # Used if bounding_box is part of CarlaEgoVehicleInfo

# Import local utility modules
from . import transforms
from . import carla_utils

class VehicleInfoPublisherNode(Node):
    """
    Publishes static information about the ego vehicle in CARLA.
    """
    def __init__(self):
        super().__init__('vehicle_info_publisher_node')

        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('ego_vehicle_role_name', 'hero')
        self.declare_parameter('update_frequency', 0.0) # Hz; <= 0.0 means publish once
        self.declare_parameter('carla_timeout', 10.0)

        # Get parameters
        self.host = self.get_parameter('carla_host').get_parameter_value().string_value
        self.port = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.role_name = self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.carla_timeout = self.get_parameter('carla_timeout').get_parameter_value().double_value
        
        self.client: carla.Client | None = None
        self.world: carla.World | None = None
        self.ego_vehicle: carla.Vehicle | None = None
        self.info_published_once: bool = False

        # Define a QoS profile for "latched" behavior (transient local durability)
        latched_qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.publisher_ = self.create_publisher(
            CarlaEgoVehicleInfo,
            f'/carla/{self.role_name}/vehicle_info',
            latched_qos_profile
        )

        # Connect to CARLA using the utility function
        self.client, self.world = carla_utils.connect_to_carla_server(
            self.get_logger(), self.host, self.port, self.carla_timeout
        )

        if not self.world:
            self.get_logger().error(f"CARLA connection failed for {self.get_name()}. "
                                    "Node will not be able to publish vehicle info.")
            # Node will still exist, but timer callback will not proceed without a world.

        # Setup timer for publishing
        # If update_frequency is <= 0, we publish once then cancel the timer.
        # The first attempt is slightly delayed to allow CARLA world to settle.
        timer_period = 1.0 / self.update_frequency if self.update_frequency > 0 else 2.0 # seconds
        self.timer = self.create_timer(timer_period, self._attempt_publish_vehicle_info)

        self.get_logger().info(f"Vehicle Info Publisher for role '{self.role_name}' initialized.")
        self.get_logger().info(f"Publishing to /carla/{self.role_name}/vehicle_info")
        if self.update_frequency > 0:
            self.get_logger().info(f"Configured publish rate: {self.update_frequency} Hz.")
        else:
            self.get_logger().info("Configured to publish once.")

    def _find_ego_vehicle(self) -> bool:
        """
        Attempts to find the ego vehicle actor in the CARLA world.
        Sets self.ego_vehicle if found and returns True, otherwise False.
        """
        if not self.world:
            self.get_logger().warn("CARLA world not available. Cannot find ego vehicle.")
            return False

        actors = self.world.get_actors()
        for actor in actors:
            if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                self.get_logger().info(f"Ego vehicle '{self.role_name}' found with ID {self.ego_vehicle.id}.")
                return True
        
        self.ego_vehicle = None # Ensure it's reset if not found in this attempt
        return False

    def _populate_vehicle_info_msg(self) -> CarlaEgoVehicleInfo | None:
        """
        Populates the CarlaEgoVehicleInfo message using data from self.ego_vehicle.
        Returns the populated message or None if data cannot be retrieved or if an error occurs.
        """
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
             self.get_logger().warn(f"Ego vehicle '{self.role_name}' is not valid or not alive for populating info.")
             self.ego_vehicle = None # Invalidate to trigger re-finding
             return None

        try:
            info_msg = CarlaEgoVehicleInfo()
            info_msg.id = self.ego_vehicle.id
            info_msg.type = self.ego_vehicle.type_id
            info_msg.rolename = self.ego_vehicle.attributes.get('role_name', self.role_name)

            physics_control = self.ego_vehicle.get_physics_control()

            for wheel_phys in physics_control.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()

                # Map available CARLA 0.10.0 WheelPhysicsControl attributes to ROS message fields
                # tire_friction
                if hasattr(wheel_phys, 'friction_force_multiplier'):
                    wheel_info.tire_friction = float(wheel_phys.friction_force_multiplier)
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'friction_force_multiplier'. Setting tire_friction to 0.0 for wheel.")
                    wheel_info.tire_friction = 0.0

                # damping_rate
                if hasattr(wheel_phys, 'suspension_damping_ratio'):
                    wheel_info.damping_rate = float(wheel_phys.suspension_damping_ratio)
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'suspension_damping_ratio'. Setting damping_rate to 0.0 for wheel.")
                    wheel_info.damping_rate = 0.0
                
                # max_steer_angle (CARLA 0.10.0 WheelPhysicsControl attribute is in degrees)
                if hasattr(wheel_phys, 'max_steer_angle'):
                    wheel_info.max_steer_angle = float(wheel_phys.max_steer_angle)
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'max_steer_angle'. Setting to 0.0 for wheel.")
                    wheel_info.max_steer_angle = 0.0

                # radius (CARLA WheelPhysicsControl attribute 'wheel_radius' is in cm)
                if hasattr(wheel_phys, 'wheel_radius'):
                    wheel_info.radius = float(wheel_phys.wheel_radius) * 0.01 # cm to m
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'wheel_radius'. Setting radius to 0.0 for wheel.")
                    wheel_info.radius = 0.0

                # max_brake_torque
                if hasattr(wheel_phys, 'max_brake_torque'):
                    wheel_info.max_brake_torque = float(wheel_phys.max_brake_torque) 
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'max_brake_torque'. Setting to 0.0 for wheel.")
                    wheel_info.max_brake_torque = 0.0
                
                # max_handbrake_torque (Check if field exists on ROS message object)
                if hasattr(wheel_info, 'max_handbrake_torque'):
                    if hasattr(wheel_phys, 'max_hand_brake_torque'): # CARLA attribute name
                        wheel_info.max_handbrake_torque = float(wheel_phys.max_hand_brake_torque)
                    else:
                        self.get_logger().warn("CARLA WheelPhysicsControl missing 'max_hand_brake_torque'. Setting ROS msg field max_handbrake_torque to 0.0 for wheel.")
                        wheel_info.max_handbrake_torque = 0.0
                else:
                    self.get_logger().warn("ROS message 'CarlaEgoVehicleInfoWheel' does not have 'max_handbrake_torque' field. Skipping for wheel.")

                # position (local offset of the wheel)
                if hasattr(wheel_phys, 'offset'): # Using 'offset' attribute
                    # Assuming 'offset' is carla.Vector3D in cm. VERIFY for CARLA 0.10.0.
                    carla_wheel_pos_offset_cm = wheel_phys.offset 
                    carla_wheel_pos_offset_m = carla.Location( 
                        x=carla_wheel_pos_offset_cm.x * 0.01, 
                        y=carla_wheel_pos_offset_cm.y * 0.01,
                        z=carla_wheel_pos_offset_cm.z * 0.01
                    )
                    ros_wheel_offset_point = transforms.carla_location_to_ros_point(carla_wheel_pos_offset_m)
                    wheel_info.position = Vector3(x=ros_wheel_offset_point.x, y=ros_wheel_offset_point.y, z=ros_wheel_offset_point.z)
                else:
                    self.get_logger().warn("CARLA WheelPhysicsControl missing 'offset'. Setting wheel position to (0,0,0).")
                    wheel_info.position = Vector3(x=0.0, y=0.0, z=0.0)

                info_msg.wheels.append(wheel_info)
            
            # General Physics Properties from VehiclePhysicsControl
            # Add hasattr checks for robustness if these attributes might change in future CARLA versions
            if hasattr(physics_control, 'max_rpm'): info_msg.max_rpm = float(physics_control.max_rpm)
            if hasattr(physics_control, 'moi'): info_msg.moi = float(physics_control.moi)
            if hasattr(physics_control, 'damping_rate_full_throttle'): info_msg.damping_rate_full_throttle = float(physics_control.damping_rate_full_throttle)
            if hasattr(physics_control, 'damping_rate_zero_throttle_clutch_engaged'): info_msg.damping_rate_zero_throttle_clutch_engaged = float(physics_control.damping_rate_zero_throttle_clutch_engaged)
            if hasattr(physics_control, 'damping_rate_zero_throttle_clutch_disengaged'): info_msg.damping_rate_zero_throttle_clutch_disengaged = float(physics_control.damping_rate_zero_throttle_clutch_disengaged)
            if hasattr(physics_control, 'use_gear_autobox'): info_msg.use_gear_autobox = bool(physics_control.use_gear_autobox)
            if hasattr(physics_control, 'gear_switch_time'): info_msg.gear_switch_time = float(physics_control.gear_switch_time)
            if hasattr(physics_control, 'clutch_strength'): info_msg.clutch_strength = float(physics_control.clutch_strength)
            if hasattr(physics_control, 'mass'): info_msg.mass = float(physics_control.mass)
            if hasattr(physics_control, 'drag_coefficient'): info_msg.drag_coefficient = float(physics_control.drag_coefficient)

            # Center of Mass
            if hasattr(physics_control, 'center_of_mass'):
                com_carla_offset_cm = physics_control.center_of_mass
                com_carla_offset_m = carla.Location( # Convert to carla.Location for the transform function
                    x=com_carla_offset_cm.x * 0.01, # Assuming cm
                    y=com_carla_offset_cm.y * 0.01,
                    z=com_carla_offset_cm.z * 0.01
                )
                info_msg.center_of_mass = transforms.carla_location_to_ros_vector3(com_carla_offset_m)
            else:
                self.get_logger().warn("VehiclePhysicsControl missing 'center_of_mass'. Setting to default (0,0,0).")
                info_msg.center_of_mass = Vector3(x=0.0, y=0.0, z=0.0) # Default if not found

            # Bounding Box: Removed as it's not in the current CarlaEgoVehicleInfo.msg
            # If it were, the logic would be:
            # carla_bbox = self.ego_vehicle.bounding_box
            # ros_bbox_shape = SolidPrimitive()
            # ros_bbox_shape.type = SolidPrimitive.BOX
            # ros_bbox_shape.dimensions = [
            #     float(carla_bbox.extent.x * 2.0), 
            #     float(carla_bbox.extent.y * 2.0), 
            #     float(carla_bbox.extent.z * 2.0)  
            # ]
            # info_msg.bounding_box = ros_bbox_shape # If 'bounding_box' field existed

            return info_msg
        
        except Exception as e: 
            self.get_logger().error(f"Unexpected error populating vehicle info: {e}\n{traceback.format_exc()}")
            return None


    def _attempt_publish_vehicle_info(self):
        """
        Timer callback. Attempts to find the vehicle and publish its information.
        Handles one-shot or periodic publishing.
        """
        if self.info_published_once and self.update_frequency <= 0.0:
            if self.timer and not self.timer.is_canceled():
                self.timer.cancel()
                # self.get_logger().info("Vehicle info was published once. Timer now cancelled.") # Can be noisy if called often before cancel
            return

        if not self.world: # Check if CARLA connection is valid
            self.get_logger().warn("CARLA world not available. Attempting to reconnect for vehicle info...")
            self.client, self.world = carla_utils.connect_to_carla_server( # Try to reconnect
                 self.get_logger(), self.host, self.port, self.carla_timeout
            )
            if not self.world: # If reconnect failed
                self.get_logger().error("Reconnect to CARLA failed. Cannot publish vehicle info.")
                return 
            
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            self.get_logger().info(f"Attempting to find ego vehicle '{self.role_name}' for info publisher...")
            if not self._find_ego_vehicle():
                self.get_logger().warn(f"Ego vehicle '{self.role_name}' not found in this tick. Will retry for info.")
                return
        
        info_msg = self._populate_vehicle_info_msg()

        if info_msg:
            self.publisher_.publish(info_msg)
            self.get_logger().info(f"Vehicle info for '{self.role_name}' published.")
            self.info_published_once = True 

            if self.update_frequency <= 0.0 and self.timer and not self.timer.is_canceled():
                self.timer.cancel() # Ensure timer is cancelled immediately after successful one-shot publish
                self.get_logger().info("One-shot vehicle info publish successful. Timer now definitively cancelled.")
        else:
            self.get_logger().warn("Failed to populate vehicle info message. Not publishing this cycle.")


    def destroy_node(self):
        """Node cleanup."""
        self.get_logger().info("Shutting down Vehicle Info Publisher Node...")
        if hasattr(self, 'timer') and self.timer and not self.timer.is_canceled(): # Check if timer exists
            self.timer.cancel()
        # Client and world are managed by carla_utils or will be garbage collected
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleInfoPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Keyboard interrupt received, shutting down vehicle info publisher.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in VehicleInfoPublisherNode: {e}\n{traceback.format_exc()}")
        else:
            # If node init fails, logger might not be available
            print(f"[FATAL] Unhandled exception during vehicle info node initialization: {e}")
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()