#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
# Ported to ROS2 by Will Varner under UGA's MSC Lab
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
#
#
# This file is loosely based on the reference architecture developed by Intel Corporation for Leidos located here
# https://github.com/41623134/carla-autoware/blob/master/catkin_ws/src/carla_autoware_bridge/src/carla_autoware_bridge/vehiclecmd_to_ackermanndrive
#
# That file has the following license and some code snippets from it may be present in this file as well and are under the same license.
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Subscribe from CARMA :autoware_msgs::VehicleCmd
    Topic: /hardware_interface/vehicle_cmd

Subscribe from CARMA :std_msgs::Float32
    Topic: /guidance/twist_filter/result/twist/lateral_jerk

Subscribe from CARMA :cav_msgs::GuidanceState
    Topic: /guidance/state

Publish to CARLA :ackermann_msgs::AckermannDrive
    Topic: /carla/{}/ackermann_cmd
"""

import rclpy
from rclpy.node import Node

# Import message types
from ackermann_msgs.msg import AckermannDrive
from autoware_msgs.msg import VehicleCmd
from cav_msgs.msg import GuidanceState
from std_msgs.msg import Float32

class CarmaToCarlaAckermannCmd(Node):
    def __init__(self):
        super().__init__('carma_to_carla_ackermann_cmd_node')

        # Initialize state variables
        self.guidance_state = GuidanceState.STARTUP
        self.init_status = True
        self.lateral_jerk = 0.0

        # Declare all parameters
        self.role_name = self.declare_parameter('role_name', 'ego_vehicle').get_parameter_value().string_value
        self.declare_parameter('init_speed', 10.0)
        self.declare_parameter('init_acceleration', 1.0)
        self.declare_parameter('init_steering_angle', 0.0)
        self.declare_parameter('init_jerk', 0.0)
        
        self.get_logger().info(f"Node 'carma_to_carla_ackermann_cmd_node' started for vehicle '{self.role_name}'.")

        # Create Publisher
        self.ackermann_cmd_pub = self.create_publisher(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd',
            1)

        # Create Subscribers
        self.vehicle_cmd_sub = self.create_subscription(
            VehicleCmd,
            '/hardware_interface/vehicle_cmd',
            self.vehicle_cmd_callback,
            1)

        self.guidance_state_sub = self.create_subscription(
            GuidanceState,
            '/guidance/state',
            self.guidance_state_callback,
            1)

        self.jerk_sub = self.create_subscription(
            Float32,
            '/guidance/twist_filter/result/twist/lateral_jerk',
            self.commanded_jerk_callback,
            1)

    def guidance_state_callback(self, guidance_state_msg):
        """
        Callback for guidance state subscribing from CARMA.
        """
        self.guidance_state = guidance_state_msg.state
        if self.guidance_state == GuidanceState.ENGAGED and self.init_status:
            self.init_status = False
            
            # Get initial command parameters
            init_speed = self.get_parameter('init_speed').get_parameter_value().double_value
            init_accel = self.get_parameter('init_acceleration').get_parameter_value().double_value
            init_steer = self.get_parameter('init_steering_angle').get_parameter_value().double_value
            init_jerk = self.get_parameter('init_jerk').get_parameter_value().double_value
            
            # Create and publish the initial command
            init_cmd = AckermannDrive()
            init_cmd.speed = init_speed
            init_cmd.acceleration = init_accel
            init_cmd.steering_angle = init_steer
            init_cmd.jerk = init_jerk
            self.ackermann_cmd_pub.publish(init_cmd)
            self.get_logger().info("Guidance engaged, publishing initial Ackermann command.")

    def commanded_jerk_callback(self, lateral_jerk_msg):
        """
        Callback for commanded jerk from CARMA.
        """
        self.lateral_jerk = lateral_jerk_msg.data

    def vehicle_cmd_callback(self, vehicle_cmd):
        """
        Callback for vehicle cmds subscribing from CARMA.
        """
        if self.guidance_state != GuidanceState.ENGAGED:
            return
        
        ackermann_drive = AckermannDrive()
        ackermann_drive.speed = vehicle_cmd.ctrl_cmd.linear_velocity
        ackermann_drive.acceleration = vehicle_cmd.ctrl_cmd.linear_acceleration
        ackermann_drive.steering_angle = vehicle_cmd.ctrl_cmd.steering_angle
        ackermann_drive.jerk = self.lateral_jerk

        self.ackermann_cmd_pub.publish(ackermann_drive)

def main(args=None):
    rclpy.init(args=args)
    node = CarmaToCarlaAckermannCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()