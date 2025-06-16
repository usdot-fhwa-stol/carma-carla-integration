#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handles posting odometry, comparable to odom_sensor.py from ROS1 bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry


class OdometrySensor(object): 

    """
    Pseudo odometry sensor
    """

    def __init__(self, parent_actor, node: Node):
        """
        Constructor
        :param parent_actor: The parent actor (ex. an EgoVehicle object) that provides the data.
        :param node: The main ROS 2 node that this publisher will use to interact with ROS.
        """
        self.parent = parent_actor
        self.node = node

        # Directly creates a ROS 2 publisher using the node handle
        self.odometry_publisher = self.node.create_publisher(
            Odometry,
            f"{self.parent.get_topic_prefix()}/odometry", # Topic name
            QoSProfile(depth=10) # Standard QoS profile
            )

    def destroy(self):
        self.node.destroy_publisher(self.odometry_publisher)

    def update(self):
        """
        Function (override) to update this object.
        This will be called periodically by the main bridge node.
        """
        # Get current timestamp from the node's clock
        timestamp = self.node.get_clock().now().to_msg()

        odometry = Odometry(header=self.parent.get_msg_header("map", timestamp=timestamp))
        odometry.child_frame_id = self.parent.name #Bypassing parent getter, can't find for some reason
        try:
            odometry.pose.pose = self.parent.get_current_ros_pose()
            odometry.twist.twist = self.parent.get_current_ros_twist_rotated()
        except AttributeError:
            # ROS 2 logger for warnings
            self.node.get_logger().warn(
                f"OdometrySensor could not publish. parent actor {self.parent.uid} not found"
            )
            return
        self.odometry_publisher.publish(odometry)
