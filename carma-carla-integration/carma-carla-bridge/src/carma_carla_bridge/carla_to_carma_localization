#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
# Migrated to ROS2 under Will Varner @ UGA MSC Lab 2025
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
# This file is loosely based on the reference architecture developed by Intel Corporation for Leidos located here
# https://github.com/41623134/carla-autoware/blob/master/catkin_ws/src/carla_autoware_bridge/src/carla_autoware_bridge/odometry_to_posestamped
#
# That file has the following license and some code snippets from it may be present in this file as well and are under the same license.
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
ROS2 migration of carla_to_carma_localization
Original path: carma-carla-integration/carma-carla-bridge/src/carma_carla_bridge/carla_to_carma_localization.py
Subscribe CARLA ROS2 Odometry to publish CARMA PoseStamped & TwistStamped for localization.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


class CarlaToCarmaLocalization(Node):
   def __init__(self):
       super().__init__('carla_to_carma_localization')
       # Declare parameter for vehicle role name
       self.declare_parameter('role_name', 'hero')
       role = self.get_parameter('role_name').get_parameter_value().string_value

       # Publishers
       self.gnss_pub  = self.create_publisher(PoseStamped,  '/localization/gnss_pose',          10)
       self.pose_pub  = self.create_publisher(PoseStamped,  '/localization/current_pose',       10)
       self.twist_pub = self.create_publisher(TwistStamped, '/hardware_interface/vehicle/twist', 10)

       # Subscription to CARLA ROS2 odometry topic
       odom_topic = f'/carla/{role}/odometry'
       self.create_subscription(
           Odometry,
           odom_topic,
           self.odometry_callback,
           10
       )
       self.get_logger().info(f'Subscribed to {odom_topic}')

   def odometry_callback(self, msg: Odometry):
       # Convert Odometry to PoseStamped for current and GNSS
       pose_msg = PoseStamped()
       pose_msg.header = msg.header
       pose_msg.pose   = msg.pose.pose
       self.pose_pub.publish(pose_msg)
       self.gnss_pub.publish(pose_msg)

       # Convert Odometry to TwistStamped
       twist_msg = TwistStamped()
       twist_msg.header = msg.header
       twist_msg.twist  = msg.twist.twist
       self.twist_pub.publish(twist_msg)

def main(args=None):
   rclpy.init(args=args)
   node = CarlaToCarmaLocalization()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
