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
ROS 2 Node to publish odometry information (pose and twist) for the
ego vehicle from CARLA, assuming CARLA's native ROS 2 mode handles
simulation ticking.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import carla
import math
import traceback # For detailed exception logging

# Import standard ROS messages
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Import local utility modules
from . import transforms
from . import carla_utils

class OdometryPublisherNode(Node):
    """
    Publishes odometry (pose in 'map' or 'odom' frame, twist in 'child_frame_id')
    of the ego vehicle by reading data from CARLA. Assumes CARLA's native
    ROS 2 mode manages the simulation tick.
    """
    def __init__(self):
        super().__init__('odometry_publisher_node')

        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('ego_vehicle_role_name', 'hero')
        self.declare_parameter('update_frequency', 20.0) # Hz
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('world_frame_id', 'map') # Typically 'map' or 'odom'
        self.declare_parameter('child_frame_id_prefix', '') # e.g., "hero/" or empty

        # Get parameters
        self.host = self.get_parameter('carla_host').get_parameter_value().string_value
        self.port = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.role_name = self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.carla_timeout = self.get_parameter('carla_timeout').get_parameter_value().double_value
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        child_frame_id_prefix = self.get_parameter('child_frame_id_prefix').get_parameter_value().string_value
        self.child_frame_id = child_frame_id_prefix + self.role_name if child_frame_id_prefix else self.role_name
        
        self.client: carla.Client | None = None
        self.world: carla.World | None = None
        self.ego_vehicle: carla.Vehicle | None = None
        
        # QoS for odometry: Reliable, Volatile (not latched)
        odometry_qos_profile = QoSProfile(
            depth=10, 
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE 
        )

        self.publisher_ = self.create_publisher(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            odometry_qos_profile
        )

        # Connect to CARLA using the utility function
        self.client, self.world = carla_utils.connect_to_carla_server(
            self.get_logger(), self.host, self.port, self.carla_timeout
        )

        if not self.world:
            self.get_logger().error(f"CARLA connection failed for {self.get_name()}. "
                                    "Node will not be able to publish odometry.")
        
        if self.world and self.update_frequency > 0:
            self.timer = self.create_timer(1.0 / self.update_frequency, self._publish_odometry)
        else:
            self.get_logger().warn(f"Timer not started for odometry. World available: {self.world is not None}, "
                                   f"Update Freq: {self.update_frequency}")

        self.get_logger().info(f"Odometry Publisher for role '{self.role_name}' initialized (Passive CARLA Mode).")
        self.get_logger().info(f"Publishing to /carla/{self.role_name}/odometry at {self.update_frequency} Hz.")
        self.get_logger().info(f"Odometry frame_id: '{self.world_frame_id}', child_frame_id: '{self.child_frame_id}'")

    def _find_ego_vehicle(self) -> bool:
        """
        Attempts to find the ego vehicle actor in the CARLA world.
        Sets self.ego_vehicle if found and returns True, otherwise False.
        """
        if not self.world:
            self.get_logger().warn("CARLA world not available. Cannot find ego vehicle for odometry.")
            return False
        try:
            for actor in self.world.get_actors():
                if isinstance(actor, carla.Vehicle) and actor.attributes.get('role_name') == self.role_name:
                    self.ego_vehicle = actor
                    # self.get_logger().debug(f"Ego vehicle '{self.role_name}' found for odometry.") # Can be spammy
                    return True
        except RuntimeError as e: 
            self.get_logger().warn(f"RuntimeError while trying to find ego vehicle for odometry: {e}. World might be changing.")
            self.ego_vehicle = None # Invalidate if error occurs during search
            return False
        
        self.ego_vehicle = None # Reset if not found
        return False

    def _publish_odometry(self):
        """
        Fetches vehicle data from CARLA and publishes it as Odometry.
        Assumes CARLA world is being ticked externally (e.g., by CARLA's native --ros2 mode).
        """
        if not self.world:
            self.get_logger().warn("No CARLA world available for odometry. Attempting to reconnect passively...")
            self.client, self.world = carla_utils.connect_to_carla_server( # Try to reconnect
                 self.get_logger(), self.host, self.port, self.carla_timeout
            )
            if not self.world: # If reconnect failed
                self.get_logger().error("Reconnect to CARLA failed. Skipping odometry publish cycle.")
                return
            
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            # self.get_logger().debug(f"Attempting to find ego vehicle '{self.role_name}' for odometry...") # Can be spammy
            if not self._find_ego_vehicle():
                # self.get_logger().warn(f"Ego vehicle '{self.role_name}' not found this cycle. Skipping odometry.") # Can be spammy
                return
        
        try:
            # Get current data from CARLA; this should reflect the latest simulation state
            actor_transform = self.ego_vehicle.get_transform()
            actor_linear_velocity = self.ego_vehicle.get_velocity()  # In world frame, m/s
            actor_angular_velocity = self.ego_vehicle.get_angular_velocity() # In vehicle local frame, deg/s

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.world_frame_id
            odom_msg.child_frame_id = self.child_frame_id

            # Populate Pose
            odom_msg.pose.pose = transforms.carla_transform_to_ros_pose(actor_transform)
            # Covariance for pose is not set here, defaults to zeros.
            # For a production system, appropriate covariance values should be estimated or configured.

            # Populate Twist
            ros_twist = transforms.carla_velocity_to_ros_twist(
                carla_linear_velocity=actor_linear_velocity,
                carla_angular_velocity=actor_angular_velocity
                # The carla_velocity_to_ros_twist function handles necessary frame
                # and unit conversions (e.g., angular velocity from deg/s to rad/s and axis mapping).
            )
            odom_msg.twist.twist.linear = ros_twist.linear
            odom_msg.twist.twist.angular = ros_twist.angular
            # Covariance for twist is not set here, defaults to zeros.
            
            self.publisher_.publish(odom_msg)
            # self.get_logger().debug("Odometry message published.") # Can be very spammy

        except RuntimeError as e: # Catch CARLA-specific errors, e.g., if actor is suddenly destroyed
            if self.ego_vehicle and not self.ego_vehicle.is_alive:
                 self.get_logger().warn(f"Ego vehicle '{self.role_name}' for odometry became invalid (likely destroyed). Will re-find.")
            else: # Other runtime errors from CARLA
                 self.get_logger().error(f"RuntimeError publishing odometry: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None # Reset to trigger re-finding
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing odometry: {e}\n{traceback.format_exc()}")
            self.ego_vehicle = None # Reset to ensure re-finding attempt

    def destroy_node(self):
        """Node cleanup."""
        self.get_logger().info(f"Shutting down {self.get_name()}...")
        if hasattr(self, 'timer') and self.timer and not self.timer.is_canceled():
            self.timer.cancel()
        # No CARLA world settings to restore as this node operates passively.
        # CARLA client connection will be closed when self.client is garbage collected.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in {node.get_name()}: {e}\n{traceback.format_exc()}")
        else:
            print(f"[FATAL] Unhandled exception during odometry node initialization: {e}")
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok(): # Check if shutdown hasn't already been called
            rclpy.shutdown()

if __name__ == '__main__':
    main()