#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians (Ported to ROS 2)
"""

from carla import WalkerControl

from .traffic_participant import TrafficParticipant

from carla_msgs.msg import CarlaWalkerControl
from derived_object_msgs.msg import Object


class Walker(TrafficParticipant):
    """
    Actor implementation details for pedestrians
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor
        """
        super(Walker, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)

        # Comment out control subscriber to focus on publishing logic for ROS2 
        # self.control_subscriber = node.create_subscription(
        #     CarlaWalkerControl,
        #     self.get_topic_prefix() + "/walker_control_cmd",
        #     self.control_command_updated,
        #     qos_profile=10)

    def destroy(self):
        """
        Function (override) to destroy this object.
        """
        super(Walker, self).destroy()
        # self.node.destroy_subscription(self.control_subscriber)

    def get_classification(self):
        """
        Function (override) to get classification. This is the key method used by ObjectSensor.
        """
        return Object.CLASSIFICATION_PEDESTRIAN

    # Commented out control logic for now
    #
    # def control_command_updated(self, ros_walker_control):
    #     """
    #     Receive a CarlaWalkerControl msg and send to CARLA
    #     """
    #     walker_control = WalkerControl()
    #     walker_control.direction.x = ros_walker_control.direction.x
    #     walker_control.direction.y = -ros_walker_control.direction.y
    #     walker_control.direction.z = ros_walker_control.direction.z
    #     walker_control.speed = ros_walker_control.speed
    #     walker_control.jump = ros_walker_control.jump
    #     self.carla_actor.apply_control(walker_control)