#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""

import sys

import numpy
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import rclpy
from rclpy.node import Node

from . import carla_control_physics as phys
from rclpy.qos import QoSProfile, DurabilityPolicy

from autoware_control_msgs.msg import Control

from std_msgs.msg import Header # pylint: disable=wrong-import-order
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_msgs.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error

from rcl_interfaces.msg import SetParametersResult


class CarlaAckermannControl(Node):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor
        """
        super().__init__("ackermann_control_node")

        # --------------------------------------------------------------------
        # PID controller
        current_time_sec = lambda: self.get_clock().now().nanoseconds / 1e9

        # Declare and get all PID parameters first using the ROS 2 standards
        speed_kp = self.declare_parameter("speed_Kp", 0.05).get_parameter_value().double_value
        speed_ki = self.declare_parameter("speed_Ki", 0.0).get_parameter_value().double_value
        speed_kd = self.declare_parameter("speed_Kd", 0.5).get_parameter_value().double_value
        accel_kp = self.declare_parameter("accel_Kp", 0.05).get_parameter_value().double_value
        accel_ki = self.declare_parameter("accel_Ki", 0.0).get_parameter_value().double_value
        accel_kd = self.declare_parameter("accel_Kd", 0.05).get_parameter_value().double_value

        # Initialize PID controllers with the ROS 2 parameters and pass the time function directly.
        self.speed_controller = PID(
            Kp=speed_kp, Ki=speed_ki, Kd=speed_kd,
            sample_time=0.05,
            output_limits=(-1., 1.),
            get_time=current_time_sec
        )
        self.accel_controller = PID(
            Kp=accel_kp, Ki=accel_ki, Kd=accel_kd,
            sample_time=0.05,
            output_limits=(-1, 1),
            get_time=current_time_sec
        )
        # --------------------------------------------------------------------

        self.add_on_set_parameters_callback(self.reconfigure_pid_parameters)
        
        self.control_loop_rate = self.declare_parameter("control_loop_rate", 0.05).get_parameter_value().double_value
        self.last_ackermann_msg_received_sec = self.get_clock().now().nanoseconds / 1e9
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = self.declare_parameter('role_name', 'ego_vehicle').get_parameter_value().string_value
        
        # control info message initialization
        self.info = EgoVehicleControlInfo()
        self.vehicle_info_updated(self.vehicle_info)
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.
        self.info.current.time_sec = self.get_clock().now().nanoseconds / 1e9
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True

        # ROS 2 Changes

        # ackermann drive commands subscriber
        self.control_subscriber = self.create_subscription(
            Control,
            "/hardware_interface/vehicle_cmd",  # This is the topic from the CARMA Platform
            self.control_command_updated, # Renamed callback
            10
        )

        # current status of the vehicle subscriber
        self.vehicle_status_subscriber = self.create_subscription(
            CarlaEgoVehicleStatus,
            "/carla/" + self.role_name + "/vehicle_status",
            self.vehicle_status_updated,
            10
        )

        # vehicle info subscriber
        self.vehicle_info_subscriber = self.create_subscription(
            CarlaEgoVehicleInfo,
            "/carla/" + self.role_name + "/vehicle_info",
            self.vehicle_info_updated,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # to send command to carla publisher
        self.carla_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            1)

        # report controller info publisher
        self.control_info_publisher = self.create_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            1)

    def reconfigure_pid_parameters(self, params):  # pylint: disable=function-redefined
        """Check and update the node's parameters."""
        param_values = {p.name: p.value for p in params}

        pid_param_names = {
            "speed_Kp",
            "speed_Ki",
            "speed_Kd",
            "accel_Kp",
            "accel_Ki",
            "accel_Kd",
        }
        common_names = pid_param_names.intersection(param_values.keys())
        if not common_names:
            return SetParametersResult(successful=True)

        if any(p.value is None for p in params):
            return SetParametersResult(
                successful=False, reason="Parameter must have a value assigned"
            )

        self.speed_controller.tunings = (
            param_values.get("speed_Kp", self.speed_controller.Kp),
            param_values.get("speed_Ki", self.speed_controller.Ki),
            param_values.get("speed_Kd", self.speed_controller.Kd),
        )
        self.accel_controller.tunings = (
            param_values.get("accel_Kp", self.accel_controller.Kp),
            param_values.get("accel_Ki", self.accel_controller.Ki),
            param_values.get("accel_Kd", self.accel_controller.Kd),
        )

        self.get_logger().info(
            "Reconfigure Request:  speed ({}, {}, {}), accel ({}, {}, {})".format(
                self.speed_controller.tunings[0],
                self.speed_controller.tunings[1],
                self.speed_controller.tunings[2],
                self.accel_controller.tunings[0],
                self.accel_controller.tunings[1],
                self.accel_controller.tunings[2]
            )
        )

        return SetParametersResult(successful=True)

    def get_msg_header(self):
        """
        Get a filled ROS message header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg() # Use the node's clock
        return header

    def vehicle_status_updated(self, vehicle_status):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        # set target values
        self.vehicle_status = vehicle_status

    def vehicle_info_updated(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.vehicle_info)
        self.info.restrictions.min_accel = self.declare_parameter('min_accel', 1.0).get_parameter_value().double_value
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def control_command_updated(self, msg: Control): # Testing correct ros2 message type
        """
        Callback for new control commands from the CARMA Platform.
        """
        self.last_ackermann_msg_received_sec = self.get_clock().now().nanoseconds / 1e9

        # Read from the fields of the Autoware Control message
        self.set_target_steering_angle(msg.lateral.steering_tire_angle)
        self.set_target_speed(msg.longitudinal.speed)
        self.set_target_accel(msg.longitudinal.acceleration)
        self.set_target_jerk(msg.longitudinal.jerk)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            self.get_logger().error("Max steering angle reached, clipping value")
            self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            self.get_logger().error("Max speed reached, clipping value")
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel
        else:
            self.info.target.accel = numpy.clip(
                target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)

    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        self.run_speed_control_loop()
        self.run_accel_control_loop()
        if not self.info.output.hand_brake:
            self.update_drive_vehicle_control_command()

            # only send out the Carla Control Command if AckermannDrive messages are
            # received in the last second (e.g. to allows manually controlling the vehicle)
            if (self.last_ackermann_msg_received_sec + 1.0) > (self.get_clock().now().nanoseconds / 1e9):
                self.info.output.header = self.get_msg_header()
                self.carla_control_publisher.publish(self.info.output)

    def control_steering(self):
        """
        Basic steering control
        """
        self.info.output.steer = self.info.target.steering_angle / \
            self.info.restrictions.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        # from this velocity on it is allowed to switch to reverse gear
        standing_still_epsilon = 0.1
        # from this velocity on hand brake is turned on
        full_stop_epsilon = 0.00001

        # auto-control of hand-brake and reverse gear
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.status.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    self.get_logger().info(
                        "VehicleControl: Change of driving direction to reverse")
                    self.info.output.reverse = True
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    self.get_logger().info(
                        "VehicleControl: Change of driving direction to forward")
                    self.info.output.reverse = False
            if self.info.target.speed_abs < full_stop_epsilon:
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.current.speed = 0.
                self.info.current.speed_abs = 0.
                self.info.current.accel = 0.
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            self.get_logger().info("VehicleControl: Request change of driving direction."
                         " v_current={} v_desired={}"
                         " Set desired speed to 0".format(self.info.current.speed,
                                                          self.info.target.speed))
            self.set_target_speed(0.)

    def run_speed_control_loop(self):
        """
        Run the PID control loop for the speed

        The speed control is only activated if desired acceleration is moderate
        otherwhise we try to follow the desired acceleration values

        Reasoning behind:

        An autonomous vehicle calculates a trajectory including position and velocities.
        The ackermann drive is derived directly from that trajectory.
        The acceleration and jerk values provided by the ackermann drive command
        reflect already the speed profile of the trajectory.
        It makes no sense to try to mimick this a-priori knowledge by the speed PID
        controller.
        =>
        The speed controller is mainly responsible to keep the speed.
        On expected speed changes, the speed control loop is disabled
        """
        epsilon = 0.00001
        target_accel_abs = abs(self.info.target.accel)
        if target_accel_abs < self.info.restrictions.min_accel:
            if self.info.status.speed_control_activation_count < 5:
                self.info.status.speed_control_activation_count += 1
        else:
            if self.info.status.speed_control_activation_count > 0:
                self.info.status.speed_control_activation_count -= 1
        # set the auto_mode of the controller accordingly
        self.speed_controller.auto_mode = self.info.status.speed_control_activation_count >= 5

        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.info.target.speed_abs
            self.info.status.speed_control_accel_delta = float(self.speed_controller(
                self.info.current.speed_abs))

            # clipping borders
            clipping_lower_border = -target_accel_abs
            clipping_upper_border = target_accel_abs
            # per definition of ackermann drive: if zero, then use max value
            if target_accel_abs < epsilon:
                clipping_lower_border = -self.info.restrictions.max_decel
                clipping_upper_border = self.info.restrictions.max_accel
            self.info.status.speed_control_accel_target = numpy.clip(
                self.info.status.speed_control_accel_target +
                self.info.status.speed_control_accel_delta,
                clipping_lower_border, clipping_upper_border)
        else:
            self.info.status.speed_control_accel_delta = 0.
            self.info.status.speed_control_accel_target = self.info.target.accel

    def run_accel_control_loop(self):
        """
        Run the PID control loop for the acceleration
        """
        # setpoint of the acceleration controller is the output of the speed controller
        self.accel_controller.setpoint = self.info.status.speed_control_accel_target
        self.info.status.accel_control_pedal_delta = float(self.accel_controller(
            self.info.current.accel))
        # @todo: we might want to scale by making use of the the abs-jerk value
        # If the jerk input is big, then the trajectory input expects already quick changes
        # in the acceleration; to respect this we put an additional proportional factor on top
        self.info.status.accel_control_pedal_target = numpy.clip(
            self.info.status.accel_control_pedal_target +
            self.info.status.accel_control_pedal_delta,
            -self.info.restrictions.max_pedal, self.info.restrictions.max_pedal)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        self.info.status.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
            self.vehicle_info, self.vehicle_status, self.info.output.reverse)

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.status.brake_upper_border = self.info.status.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.vehicle_info)

        if self.info.status.accel_control_pedal_target > self.info.status.throttle_lower_border:
            self.info.status.status = "accelerating"
            self.info.output.brake = 0.0
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            self.info.output.throttle = (
                (self.info.status.accel_control_pedal_target -
                 self.info.status.throttle_lower_border) /
                abs(self.info.restrictions.max_pedal))
        elif self.info.status.accel_control_pedal_target > self.info.status.brake_upper_border:
            self.info.status.status = "coasting"
            # no control required
            self.info.output.brake = 0.0
            self.info.output.throttle = 0.0
        else:
            self.info.status.status = "braking"
            # braking required
            self.info.output.brake = (
                (self.info.status.brake_upper_border -
                 self.info.status.accel_control_pedal_target) /
                abs(self.info.restrictions.max_pedal))
            self.info.output.throttle = 0.0

        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(
            self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(
            self.info.output.throttle, 0., 1.)

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.control_info_publisher.publish(self.info)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        delta_time = current_time_sec - self.info.current.time_sec
        current_speed = self.vehicle_status.velocity
        if delta_time > 0:
            delta_speed = current_speed - self.info.current.speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def run(self):
        """

        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.update_current_values()
            self.vehicle_control_cycle()
            self.send_ego_vehicle_control_info_msg()

        self.create_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = CarlaAckermannControl()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals() and controller:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
