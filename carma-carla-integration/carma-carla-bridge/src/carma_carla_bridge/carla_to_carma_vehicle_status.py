#!/usr/bin/env python3
# Copyright (C) 2021 LEIDOS.
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
# https://github.com/41623134/carla-autoware/blob/master/catkin_ws/src/carla_autoware_bridge/src/carla_autoware_bridge/carla_to_autoware_vehicle_status
#
# That file has the following license and some code snippets from it may be present in this file as well and are under the same license.
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Subscribe from CARLA :carla_msgs::CarlaEgoVehicleStatus
    Topic: /carla/{}/vehicle_status
Subscribe from CARLA :carla_msgs::CarlaEgoVehicleInfo
    Topic: /carla/{}/vehicle_info
Publish to CARMA :autoware_msgs::VehicleStatus
    Topic: /hardware_interface/vehicle_status
"""
import math
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleStatus
from autoware_msgs.msg import VehicleStatus, Gear

class CarlaToCarmaVehicleStatus(Node):
    def __init__(self):
        super().__init__('carla_to_carma_vehicle_status')
        # Declare parameter for vehicle role name and max steering angle
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('max_steering_degree', 70.0)
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.max_steering_angle = math.radians(self.get_parameter('max_steering_degree').get_parameter_value().double_value)
        
        # Initialize vehicle info storage
        self.vehicle_info = None
        
        # Create publisher
        self.vehicle_status_pub = self.create_publisher(VehicleStatus, '/hardware_interface/vehicle_status', 10)
        
        # Create subscribers
        self.vehicle_status_sub = self.create_subscription(
            CarlaEgoVehicleStatus,
            f'/carla/{self.role_name}/vehicle_status',
            self.vehicle_status_callback,
            10
        )
        self.vehicle_info_sub = self.create_subscription(
            CarlaEgoVehicleInfo,
            f'/carla/{self.role_name}/vehicle_info',
            self.vehicle_info_callback,
            10
        )
        
        self.get_logger().info(f'Subscribed to /carla/{self.role_name}/vehicle_status and /carla/{self.role_name}/vehicle_info')
        self.get_logger().info('Publishing to /hardware_interface/vehicle_status')

    def vehicle_info_callback(self, vehicle_info_msg: CarlaEgoVehicleInfo):
        """
        Callback for vehicle info
        vehicle_info_msg type: carla_msgs::CarlaEgoVehicleInfo
        """
        self.vehicle_info = vehicle_info_msg

    def vehicle_status_callback(self, vehicle_status_msg: CarlaEgoVehicleStatus):
        """
        Callback for vehicle status
        vehicle_status_msg type: carla_msgs::CarlaEgoVehicleStatus
        """
        if self.vehicle_info is None:
            self.get_logger().warn('Vehicle info not received yet, skipping status publish')
            return

        status = VehicleStatus()
        status.header = vehicle_status_msg.header
        
        # Calculate max steering angle (use smallest non-zero value of all wheels)
        max_steering_angle = self.max_steering_angle
        for wheel in self.vehicle_info.wheels:
            if wheel.max_steer_angle and wheel.max_steer_angle < max_steering_angle:
                max_steering_angle = wheel.max_steer_angle
        
        status.angle = vehicle_status_msg.control.steer * math.degrees(max_steering_angle)
        status.speed = vehicle_status_msg.velocity
        
        # Set gear based on reverse control
        # Corrected code
        carla_gear = vehicle_status_msg.control.gear

        if carla_gear > 0:
            # Any positive number from CARLA is Drive
            status.current_gear.gear = 4 # DRIVE for VehicleStatus message
        elif carla_gear == 0:
            # Zero is Neutral
            status.current_gear.gear = 2 # NEUTRAL for VehicleStatus message
        else: # carla_gear < 0
            # Any negative number from CARLA is Reverse
            status.current_gear.gear = 1 # REVERSE for VehicleStatus message
        
        # Set drive mode based on manual gear shift
        if vehicle_status_msg.control.manual_gear_shift:
            status.drivemode = VehicleStatus.MODE_MANUAL
        else:
            status.drivemode = VehicleStatus.MODE_AUTO
        
        self.vehicle_status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaToCarmaVehicleStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()