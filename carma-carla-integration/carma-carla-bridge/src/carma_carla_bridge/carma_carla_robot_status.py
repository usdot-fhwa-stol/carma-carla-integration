#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
# Ported to ROS2 by Will Varner @ UGA MSC Lab
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
Publish to CARMA: cav_msgs::RobotEnabled
Topic: /hardware_interface/controller/robot_status

Subscribe from CARMA: cav_msgs::GuidanceState
Topic: /guidance/state
"""
import rclpy
from rclpy.node import Node
from carma_driver_msgs.msg import RobotEnabled
from carma_planning_msgs.msg import GuidanceState

class RobotStatusNode(Node):
    """
    The RobotStatusNode class encapsulates the logic for monitoring CARMA's guidance
    state and publishing the robot's operational status.
    """
    def __init__(self):
        """
        Initializes the node, publisher, subscriber, and a timer for periodic publishing.
        """
        super().__init__('carma_carla_robot_status')

        # Declare parameters for the publisher rate
        self.declare_parameter('robot_status_pub_rate', 10)

        # Create the publisher for the robot's status
        self.robot_status_pub = self.create_publisher(
            RobotEnabled,
            '/hardware_interface/controller/robot_status',
            10)

        # Create the subscriber for CARMA's guidance state
        self.guidance_state_sub = self.create_subscription(
            GuidanceState,
            '/guidance/state',
            self.guidance_state_callback,
            1) # QoS depth of 1 is sufficient

        # Initialize the message to be published
        self.robot_status_msg = RobotEnabled()
        self.robot_status_msg.robot_enabled = True # This is always true as per the original logic
        self.robot_status_msg.robot_active = False # Default to not active

        # Create a timer to publish the status at a regular interval
        pub_rate = self.get_parameter('robot_status_pub_rate').value
        self.timer = self.create_timer(1.0 / pub_rate, self.publish_status)

        self.get_logger().info("carma_carla_robot_status node has started.")

    def guidance_state_callback(self, msg):
        """
        Callback for the /guidance/state subscriber. Updates the robot_active status.
        """
        # Check if the guidance state is ENGAGED or ACTIVE
        if msg.state == GuidanceState.ENGAGED or msg.state == GuidanceState.ACTIVE:
            self.robot_status_msg.robot_active = True
        else:
            self.robot_status_msg.robot_active = False

    def publish_status(self):
        """
        Timer callback to publish the current robot status.
        """
        self.robot_status_pub.publish(self.robot_status_msg)

def main(args=None):
    """
    Main entry point for the node.
    """
    rclpy.init(args=args)
    robot_status_node = RobotStatusNode()
    rclpy.spin(robot_status_node)

    # Destroy the node explicitly (garbage collector will also handle it)
    robot_status_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()