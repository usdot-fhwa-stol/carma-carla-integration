#!/usr/bin/env python
# Copyright (C) 2023 LEIDOS.
# Migrated to ROS2 under Ryan Fleming @ UGA MSC Lab 2025
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
Publish to CARMA :cav_msgs::DriverStatus
    Topic: /hardware_interface/driver_discovery;
"""
import rclpy
from rclpy.node import Node
from cav_msgs.msg import DriverStatus
from std_msgs.msg import Header

import traceback

class DriverStatusNode(Node):
    def __init__(self):
        super().__init__('carma_carla_driver_status')

        # Parameter declarations
        self.declare_parameter('driver_status_pub_rate', 10)
        self.declare_parameter('lidar_enabled', True)
        self.declare_parameter('controller_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('gnss_enabled', True)

        # Read parameters
        self.pub_rate = self.get_parameter('driver_status_pub_rate').value
        self.lidar_enabled = self.get_parameter('lidar_enabled').value
        self.controller_enabled = self.get_parameter('controller_enabled').value
        self.camera_enabled = self.get_parameter('camera_enabled').value
        self.gnss_enabled = self.get_parameter('gnss_enabled').value

        # Create publisher
        self.publisher = self.create_publisher(DriverStatus, '/hardware_interface/driver_discovery', 10)

        # Create clock
        self.clock = self.get_clock()

        self.get_logger().info(f"{self.get_name()} starting with pub rate: {self.pub_rate}")
        # Create timer with specified rate and callback
        self.timer = self.create_timer(1.0 / self.pub_rate, self.publish_driver_status)

    def publish_driver_status(self):
        msgs = [
            self.create_driver_status_msg(
                name        = '/hardware_interface/carla_driver',
                status      = DriverStatus.OPERATIONAL,
                lidar       = self.lidar_enabled,
                controller  = self.controller_enabled,
                camera      = self.camera_enabled,
                gnss        = self.gnss_enabled
            ),
            self.create_driver_status_msg(
                name        = '/hardware_interface/carla_camera_driver',
                status      = DriverStatus.OPERATIONAL,
                camera      = self.camera_enabled
            ),
            self.create_driver_status_msg(
                name        = '/hardware_interface/carla_lidar_driver',
                status      = DriverStatus.OPERATIONAL,
                lidar       = self.lidar_enabled
            ),
            self.create_driver_status_msg(
                name        = '/hardware_interface/carla_gnss_driver',
                status      = DriverStatus.OPERATIONAL,
                gnss        = self.gnss_enabled
            )
        ]

        for msg in msgs:
            self.publisher.publish(msg)

    # status value should only be a value described by DriverStatus status enumeration:
    # uint8  OFF=0, uint8  OPERATIONAL=1, uint8  DEGRADED=2, uint8  FAULT=3
    def create_driver_status_msg(self, name: str, status: int, **categories) -> DriverStatus:
        msg = DriverStatus()

        # Set msg header
        msg.header = Header()
        msg.header.stamp = self.clock.now().to_msg()
        msg.header.frame_id = ''

        msg.name = name
        msg.status = status

        for key, value in categories.items():
            if hasattr(msg, key):
                setattr(msg, key, value)
            else:
                raise AttributeError(f"DriverStatus has no field named '{key}'")

        return msg

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DriverStatusNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        # Ensure logger is available or use print
        logger = rclpy.logging.get_logger("carma_carla_driver_status_node_main")
        if node : logger = node.get_logger()
        logger.error(f"Unhandled exception in {node.get_name() if node else 'DriverStatusNode'}: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()