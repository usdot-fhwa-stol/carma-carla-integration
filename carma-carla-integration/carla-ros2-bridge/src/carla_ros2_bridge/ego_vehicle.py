#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math
import os

import numpy
import carla
from carla import VehicleControl

from . import transforms as trans

from rclpy.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.vehicle import Vehicle

from carla_msgs.msg import (
    CarlaEgoVehicleInfo,
    CarlaEgoVehicleInfoWheel,
    CarlaEgoVehicleControl,
    CarlaEgoVehicleStatus
)
from std_msgs.msg import Bool  # pylint: disable=import-error
from std_msgs.msg import ColorRGBA  # pylint: disable=import-error


class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, uid, name, parent, node, carla_actor, vehicle_control_applied_callback):
        """
        Constructor for Ros2

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle (now an rclpy.node.Node)
        :type node: rclpy.node.Node
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        # super() call remains, as we will also port the parent 'Vehicle' class
        super(EgoVehicle, self).__init__(uid=uid,
                                         name=name,
                                         parent=parent,
                                         node=node,
                                         carla_actor=carla_actor)
        self.vehicle_info_published = False
        self.vehicle_control_override = False
        self._vehicle_control_applied_callback = vehicle_control_applied_callback

        # Updated to use node.create_publisher for ROS2 design
        self.vehicle_status_publisher = node.create_publisher(
            CarlaEgoVehicleStatus,
            self.get_topic_prefix() + "/vehicle_status",
            qos_profile=10)

        # Uses node.create_publisher and a QoS profile for "latched" behavior
        self.vehicle_info_publisher = node.create_publisher(
            CarlaEgoVehicleInfo,
            self.get_topic_prefix() + "/vehicle_info",
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # --- PORTING DEBUG Comment out control subscribers to focus on publishing logic first ---
        # The creation of subscribers will be ported in a later step i think
        #
        # self.control_subscriber = node.create_subscription(
        #     CarlaEgoVehicleControl,
        #     self.get_topic_prefix() + "/vehicle_control_cmd",
        #     lambda data: self.control_command_updated(data, manual_override=False),
        #     qos_profile=10)
        #
        # self.manual_control_subscriber = node.create_subscription(
        #     CarlaEgoVehicleControl,
        #     self.get_topic_prefix() + "/vehicle_control_cmd_manual",
        #     lambda data: self.control_command_updated(data, manual_override=True),
        #     qos_profile=10)
        #
        # self.control_override_subscriber = node.create_subscription(
        #     Bool,
        #     self.get_topic_prefix() + "/vehicle_control_manual_override",
        #     self.control_command_override,
        #     qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        #
        # self.enable_autopilot_subscriber = node.create_subscription(
        #     Bool,
        #     self.get_topic_prefix() + "/enable_autopilot",
        #     self.enable_autopilot_updated,
        #     qos_profile=10)

    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.0
        color.g = 255.0
        color.b = 0.0
        return color

    def send_vehicle_msgs(self, timestamp):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_status = CarlaEgoVehicleStatus(
            header=self.get_msg_header("map", timestamp=timestamp))
        vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_status.acceleration.linear = self.get_current_ros_accel().linear
        vehicle_status.orientation = self.get_current_ros_pose().orientation
        vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        vehicle_status.control.steer = self.carla_actor.get_control().steer
        vehicle_status.control.brake = self.carla_actor.get_control().brake
        vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        vehicle_status.control.gear = self.carla_actor.get_control().gear
        vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        self.vehicle_status_publisher.publish(vehicle_status)

        # only send vehicle once (in latched-mode)
        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            vehicle_info = CarlaEgoVehicleInfo()
            vehicle_info.id = self.carla_actor.id
            vehicle_info.type = self.carla_actor.type_id
            vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
            vehicle_physics = self.carla_actor.get_physics_control()

            # This loop is now adapted from your own working bridge code.
            # This loop is now corrected using the actual attributes from CARLA 0.10.0
            for wheel_phys in vehicle_physics.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()

                # Use the correct attribute names we found in the debug log
                wheel_info.tire_friction = float(wheel_phys.friction_force_multiplier)
                wheel_info.damping_rate = float(wheel_phys.suspension_damping_ratio)
                wheel_info.max_steer_angle = math.radians(wheel_phys.max_steer_angle)

                # The 'wheel_radius' attribute is in centimeters, so we convert to meters
                wheel_info.radius = float(wheel_phys.wheel_radius) * 0.01

                wheel_info.max_brake_torque = float(wheel_phys.max_brake_torque)

                # The 'offset' attribute gives the wheel's location relative to the vehicle's center (in cm)
                carla_wheel_pos_offset_m = carla.Location(
                    x=wheel_phys.offset.x * 0.01,
                    y=wheel_phys.offset.y * 0.01,
                    z=wheel_phys.offset.z * 0.01
                )
                ros_wheel_offset_point = trans.carla_location_to_ros_point(carla_wheel_pos_offset_m)
                wheel_info.position.x = ros_wheel_offset_point.x
                wheel_info.position.y = ros_wheel_offset_point.y
                wheel_info.position.z = ros_wheel_offset_point.z

                vehicle_info.wheels.append(wheel_info)

            vehicle_info.max_rpm = float(vehicle_physics.max_rpm)

            # The 'moi' attribute is now 'inertia_tensor_scale'
            vehicle_info.moi = float(vehicle_physics.inertia_tensor_scale.z) # Using Z component as a sensible default for yaw inertia

            # The 'use_gear_autobox' attribute is now 'use_automatic_gears'
            vehicle_info.use_gear_autobox = bool(vehicle_physics.use_automatic_gears)

            # The 'gear_switch_time' attribute is now 'gear_change_time'
            vehicle_info.gear_switch_time = float(vehicle_physics.gear_change_time)

            # The following attributes no longer exist so we can just remove but keep for now ig
            # vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
            # vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
            #     vehicle_physics.damping_rate_zero_throttle_clutch_engaged
            # vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
            #     vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
            # vehicle_info.clutch_strength = vehicle_physics.clutch_strength

            # These attributes still exist with the same names
            vehicle_info.mass = float(vehicle_physics.mass)
            vehicle_info.drag_coefficient = float(vehicle_physics.drag_coefficient)
            vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
            vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
            vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

            self.vehicle_info_publisher.publish(vehicle_info)

    def update(self, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        super(EgoVehicle, self).update(timestamp)
        self.send_vehicle_msgs(timestamp)
        

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS publishers and subscriptions.
        Finally forward call to super class.
        """
        # Use the node's logger
        self.node.get_logger().debug(f"Destroying EgoVehicle(id={self.get_id()})")

        # --- Temporarily disable destroying control subscribers as they are not created in __init__ ---
        # self.node.destroy_subscription(self.control_subscriber)
        # self.node.destroy_subscription(self.enable_autopilot_subscriber)
        # self.node.destroy_subscription(self.control_override_subscriber)
        # self.node.destroy_subscription(self.manual_control_subscriber)

        # Destroy the publishers created in __init__
        self.node.destroy_publisher(self.vehicle_status_publisher)
        self.node.destroy_publisher(self.vehicle_info_publisher)
        
        # Forward call
        Vehicle.destroy(self)

    '''
    ROS2 Debug. Commenting these out as well
    def control_command_override(self, enable):
        """
        Set the vehicle control mode according to ros topic
        """
        self.vehicle_control_override = enable.data

    def control_command_updated(self, ros_vehicle_control, manual_override):
        """
        Receive a CarlaEgoVehicleControl msg and send to CARLA

        This function gets called whenever a ROS CarlaEgoVehicleControl is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param manual_override: manually override the vehicle control command
        :param ros_vehicle_control: current vehicle control input received via ROS
        :type ros_vehicle_control: carla_msgs.msg.CarlaEgoVehicleControl
        :return:
        """
        if manual_override == self.vehicle_control_override:
            vehicle_control = VehicleControl()
            vehicle_control.hand_brake = ros_vehicle_control.hand_brake
            vehicle_control.brake = ros_vehicle_control.brake
            vehicle_control.steer = ros_vehicle_control.steer
            vehicle_control.throttle = ros_vehicle_control.throttle
            vehicle_control.reverse = ros_vehicle_control.reverse
            vehicle_control.manual_gear_shift = ros_vehicle_control.manual_gear_shift
            vehicle_control.gear = ros_vehicle_control.gear
            self.carla_actor.apply_control(vehicle_control)
            self._vehicle_control_applied_callback(self.get_id())

     def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        self.node.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
        self.carla_actor.set_autopilot(enable_auto_pilot.data)
    '''


    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed
