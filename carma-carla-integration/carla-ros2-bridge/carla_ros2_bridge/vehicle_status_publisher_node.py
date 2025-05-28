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
ROS 2 Node to publish dynamic vehicle status information (speed, acceleration,
orientation, control inputs) for the ego vehicle from CARLA. Assumes CARLA's
native ROS 2 mode handles simulation ticking.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import carla
import math
import traceback # For detailed exception logging

# Import the custom CARLA messages
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleControl

# Import standard ROS messages
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion

# Import local utility modules
from . import transforms
from . import carla_utils

class VehicleStatusPublisherNode(Node):
    """
    Publishes dynamic status of the ego vehicle in CARLA, including speed,
    acceleration, orientation, and current control inputs. Assumes CARLA's
    native ROS 2 mode manages the simulation tick.
    """
    def __init__(self):
        super().__init__('vehicle_status_publisher_node')

        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('ego_vehicle_role_name', 'hero')
        self.declare_parameter('update_frequency', 20.0) # Hz
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
        
        # QoS for status: Reliable, Volatile.
        status_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(
            CarlaEgoVehicleStatus,
            f'/carla/{self.role_name}/vehicle_status',
            status_qos_profile
        )

        # Connect to CARLA using the utility function
        self.client, self.world = carla_utils.connect_to_carla_server(
            self.get_logger(), self.host, self.port, self.carla_timeout
        )

        if not self.world:
            self.get_logger().error(f"CARLA connection failed for {self.get_name()}. "
                                    "Node will not be able to publish vehicle status.")

        if self.world and self.update_frequency > 0:
            self.timer = self.create_timer(1.0 / self.update_frequency, self._publish_vehicle_status)
        else:
            self.get_logger().warn(f"Timer not started for vehicle status. World available: {self.world is not None}, "
                                   f"Update Freq: {self.update_frequency}")

        self.get_logger().info(f"Vehicle Status Publisher for role '{self.role_name}' initialized (Passive CARLA Mode).")
        self.get_logger().info(f"Publishing to /carla/{self.role_name}/vehicle_status at {self.update_frequency} Hz.")

    def _find_ego_vehicle(self) -> bool:
        """
        Attempts to find the ego vehicle actor in the CARLA world.
        Sets self.ego_vehicle if found and returns True, otherwise False.
        """
        if not self.world:
            self.get_logger().warn("CARLA world not available. Cannot find ego vehicle for status.")
            return False
        try:
            for actor in self.world.get_actors():
                if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') == self.role_name:
                    self.ego_vehicle = actor
                    # self.get_logger().debug(f"Ego vehicle '{self.role_name}' found for status.") # Can be spammy
                    return True
        except RuntimeError as e: 
            self.get_logger().warn(f"RuntimeError while trying to find ego vehicle for status: {e}. World might be changing.")
            self.ego_vehicle = None
            return False
        
        self.ego_vehicle = None # Reset if not found
        return False

    def _publish_vehicle_status(self):
        """
        Fetches vehicle status from CARLA and publishes it.
        Assumes CARLA world is being ticked externally.
        """
        if not self.world:
            self.get_logger().warn("No CARLA world available for vehicle status. Attempting to reconnect passively...")
            self.client, self.world = carla_utils.connect_to_carla_server( # Try to reconnect
                 self.get_logger(), self.host, self.port, self.carla_timeout
            )
            if not self.world: # If reconnect failed
                self.get_logger().error("Reconnect to CARLA failed. Skipping vehicle status publish cycle.")
                return
            
        # No self.world.tick() here - assuming CARLA --ros2 handles it.

        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            # self.get_logger().debug(f"Attempting to find ego vehicle '{self.role_name}' for status...") # Spammy
            if not self._find_ego_vehicle():
                # self.get_logger().warn(f"Ego vehicle '{self.role_name}' not found this cycle. Skipping status.") # Spammy
                return
        
        try:
            status_msg = CarlaEgoVehicleStatus()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = self.role_name # Or a more specific frame like 'hero/base_link'

            # Velocity (scalar magnitude)
            velocity_vector = self.ego_vehicle.get_velocity() # carla.Vector3D m/s
            status_msg.velocity = math.sqrt(velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2)

            # Acceleration
            carla_accel = self.ego_vehicle.get_acceleration() # carla.Vector3D m/s^2
            ros_accel_msg = transforms.carla_acceleration_to_ros_accel(carla_accel)
            status_msg.acceleration.linear = ros_accel_msg.linear
            # status_msg.acceleration.angular is typically zero from this source

            # Orientation
            carla_transform = self.ego_vehicle.get_transform()
            status_msg.orientation = transforms.carla_rotation_to_ros_quaternion(carla_transform.rotation)
            
            # Control Inputs
            carla_control = self.ego_vehicle.get_control() # carla.VehicleControl
            ros_vehicle_control = CarlaEgoVehicleControl()
            ros_vehicle_control.throttle = float(carla_control.throttle)
            ros_vehicle_control.steer = float(carla_control.steer)
            ros_vehicle_control.brake = float(carla_control.brake)
            ros_vehicle_control.hand_brake = bool(carla_control.hand_brake)
            ros_vehicle_control.reverse = bool(carla_control.reverse)
            ros_vehicle_control.gear = int(carla_control.gear)
            ros_vehicle_control.manual_gear_shift = bool(carla_control.manual_gear_shift)
            status_msg.control = ros_vehicle_control
            
            self.publisher_.publish(status_msg)
            # self.get_logger().debug("Vehicle status message published.") # Can be spammy

        except RuntimeError as e: # Catch CARLA-specific errors
            if self.ego_vehicle and not self.ego_vehicle.is_alive:
                 self.get_logger().warn(f"Ego vehicle '{self.role_name}' for status became invalid. Will re-find.")
            else:
                 self.get_logger().error(f"RuntimeError publishing vehicle status: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None 
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing vehicle status: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None 

    def destroy_node(self):
        """Node cleanup."""
        self.get_logger().info(f"Shutting down {self.get_name()}...")
        if hasattr(self, 'timer') and self.timer and not self.timer.is_canceled():
            self.timer.cancel()
        # No CARLA world settings to restore.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleStatusPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in {node.get_name()}: {e}\n{traceback.format_exc()}")
        else:
            print(f"[FATAL] Unhandled exception during vehicle status node initialization: {e}")
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()