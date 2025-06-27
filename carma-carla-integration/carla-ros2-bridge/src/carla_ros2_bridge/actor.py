#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle Actor objects (Ported to ROS 2)
"""
from rclpy.node import Node
from std_msgs.msg import Header

# Use a local import for the transforms module, originally from Carla_Common but simplifying overall porting size
from . import transforms as trans


class Actor(object):
    """
    Generic base class for all carla actors, serving as the foundation for the bridge.
    It has been simplified by merging the functionality of the original 'PseudoActor'
    and 'Actor' classes.
    """

    def __init__(self, uid, name, parent, node: Node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :param name: name identiying this object
        :param parent: the parent of this (unused in base class)
        :param node: the ROS 2 node handle
        :param carla_actor: carla actor object
        """
        self.uid = uid
        self.name = name
        self.parent = parent
        self.node = node
        self.carla_actor = carla_actor
        self.carla_actor_id = carla_actor.id

        # Stored state updated from the update() method
        self._current_ros_transform = None
        self._current_ros_pose = None
        self._current_ros_twist = None
        self._current_ros_accel = None

    def destroy(self):
        """
        Function to destroy this object.
        Removes the reference to the carla.Actor object.
        """
        self.carla_actor = None

    def get_id(self):
        """
        Getter for the carla_id of this.
        :return: unique carla_id of this object
        """
        return self.carla_actor_id
    
    def get_prefix(self):
        """
        Getter for the actor's role name, used as a prefix for frames and topics.
        :return: role_name of this actor
        """
        return self.name

    def get_topic_prefix(self):
        """
        Get the topic prefix for this actor.
        :return: "/carla/{role_name}"
        """
        # The 'name' is expected to be the role_name
        return "/carla/" + self.name

    def get_msg_header(self, frame_id=None, timestamp=None):
        """
        Get a ROS message header
        """
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        if timestamp:
            header.stamp = timestamp
        else:
            header.stamp = self.node.get_clock().now().to_msg()
        return header

    def update(self, timestamp):
        """
        This is the critical update function.
        It is called by the main bridge loop every frame.
        It caches the actor's current state (pose, twist, accel)
        so that it's only calculated once per frame.
        """
        # Get the new transform and velocity from the CARLA actor
        carla_transform = self.carla_actor.get_transform()
        carla_velocity = self.carla_actor.get_velocity()
        carla_angular_velocity = self.carla_actor.get_angular_velocity()
        carla_acceleration = self.carla_actor.get_acceleration()

        # Convert and cache the ROS messages
        self._current_ros_transform = trans.carla_transform_to_ros_transform(carla_transform)
        self._current_ros_pose = trans.carla_transform_to_ros_pose(carla_transform)
        self._current_ros_twist = trans.carla_velocity_to_ros_twist(carla_velocity, carla_angular_velocity)
        self._current_ros_accel = trans.carla_acceleration_to_ros_accel(carla_acceleration)

    # --- Getter methods for the cached state ---

    def get_current_ros_pose(self):
        """
        Function to provide the current ROS pose from the cached state.
        :return: the ROS pose of this actor
        """
        return self._current_ros_pose

    def get_current_ros_transform(self):
        """
        Function to provide the current ROS transform from the cached state.
        :return: the ROS transform of this actor
        """
        return self._current_ros_transform

    def get_current_ros_twist(self):
        """
        Function to provide the current ROS twist from the cached state.
        :return: the ROS twist of this actor
        """
        return self._current_ros_twist

    def get_current_ros_twist_rotated(self):
        """
        Function to provide the current ROS twist, rotated to the vehicle's frame.
        (This method remains for compatibility with OdometrySensor).
        """
        # This is one of the few methods that might still do a direct conversion
        # if the rotation needs to be applied dynamically.
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

    def get_current_ros_accel(self):
        """
        Function to provide the current ROS accel from the cached state.
        :return: the ROS accel of this actor
        """
        return self._current_ros_accel