#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handles publishing of detected objects in the world (Ported to ROS 2)
"""
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .vehicle import Vehicle
from .walker import Walker

from derived_object_msgs.msg import ObjectArray


class ObjectSensor(object):
    """
    Pseudo object sensor, responsible for publishing an ObjectArray of detected actors.
    """

    def __init__(self, parent_actor, node: Node, actor_list):
        """
        Constructor

        :param parent_actor: The parent actor (ego vehicle) that this sensor is attached to.
        :param node: The main ROS 2 node handle.
        :param actor_list: The dictionary of all current actors managed by the bridge.
        """
        self.parent = parent_actor
        self.node = node
        self.actor_list = actor_list

        # Uses node.create_publisher and a clear topic name for ROS 2
        self.object_publisher = self.node.create_publisher(
            ObjectArray,
            f"{self.parent.get_topic_prefix()}/objects", # e.g., /carla/hero/objects
            QoSProfile(depth=10))

    def destroy(self):
        """
        Function to destroy this object.
        """
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    def update(self, timestamp=None):
        """
        Function to update this object.
        On update, it iterates through all actors, creates an ObjectArray, and publishes it.
        """
        # Gets timestamp from the node's clock now
        if timestamp is None:            
            timestamp = self.node.get_clock().now().to_msg()

        ros_objects = ObjectArray()
        # The header is created using the parent's (ego vehicle's) methods
        ros_objects.header = self.parent.get_msg_header(frame_id="map", timestamp=timestamp)
        
        for actor_id, actor in list(self.actor_list.items()):
            # Exclude the ego vehicle itself from the list of detected objects
            if self.parent is None or self.parent.uid != actor_id:
                # The core logic here works because our Vehicle and Walker classes
                # inherit from TrafficParticipant, which has get_object_info().
                if isinstance(actor, Vehicle):
                    ros_objects.objects.append(actor.get_object_info())
                elif isinstance(actor, Walker):
                    ros_objects.objects.append(actor.get_object_info())
        
        self.object_publisher.publish(ros_objects)