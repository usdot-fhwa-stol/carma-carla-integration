#!/usr/bin/env python3
# log_carma_carla_controls.py
#
# Logs to CSV:
#  - Ackermann commands from CARMA (AckermannDriveStamped) on /carla/{role_name}/ackermann_cmd
#  - Vehicle control commands to CARLA (CarlaEgoVehicleControl) on /carla/{role_name}/vehicle_control_cmd
#  - Actual ego state from CARLA (CarlaEgoVehicleStatus) on /carla/{role_name}/vehicle_status
#  - Pose/speed/yaw from Odometry fallback on /carla/{role_name}/odometry
#  - First CSV column is sim time seconds (ROS clock, honors use_sim_time)

import csv
import math
import os
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# From the CARLA ROS bridge
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus


def quat_to_yaw(q: Quaternion) -> float:
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def fmt(x):
    return f"{x:.6f}" if isinstance(x, float) else (x if x is not None else "")


class CarmaCarlaControlLogger(Node):
    def __init__(self):
        super().__init__('carma_carla_control_logger')

        # Core params
        self.declare_parameter('role_name', 'carma_1')
        self.declare_parameter('carma_ackermann_topic', '')
        self.declare_parameter('carla_control_topic', '')
        self.declare_parameter('carla_status_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('csv_path', '/tmp/carma_carla_controls_log.csv')
        self.declare_parameter('flush_every', 10)

        role = self.get_parameter('role_name').get_parameter_value().string_value

        # Compute default topics to match the CARLA bridge layout
        def_topic_carma_ack   = f"/carla/{role}/ackermann_cmd"
        def_topic_carla_ctrl  = f"/carla/{role}/vehicle_control_cmd"
        def_topic_status      = f"/carla/{role}/vehicle_status"
        def_topic_odom        = f"/carla/{role}/odometry"

        # Allow explicit overrides
        carma_ackermann_topic = self._get_or_default_str('carma_ackermann_topic', def_topic_carma_ack)
        carla_control_topic   = self._get_or_default_str('carla_control_topic',   def_topic_carla_ctrl)
        self.carla_status_topic = self._get_or_default_str('carla_status_topic',  def_topic_status)
        self.odom_topic         = self._get_or_default_str('odom_topic',          def_topic_odom)
        self.csv_path           = self.get_parameter('csv_path').get_parameter_value().string_value
        self.flush_every        = int(self.get_parameter('flush_every').get_parameter_value().integer_value or 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=30
        )

        # Subscriptions
        self.sub_carma  = self.create_subscription(AckermannDrive, carma_ackermann_topic, self.on_carma_ackermann, qos)
        self.sub_carla  = self.create_subscription(CarlaEgoVehicleControl, carla_control_topic, self.on_carla_control, qos)
        self.sub_status = self.create_subscription(CarlaEgoVehicleStatus, self.carla_status_topic, self.on_carla_status, qos)
        self.sub_odom   = self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)

        # Last-seen data
        self.last_carma: Optional[AckermannDrive] = None
        self.last_carla: Optional[CarlaEgoVehicleControl] = None
        self.last_status: Optional[CarlaEgoVehicleStatus] = None
        self.last_odom: Optional[Odometry] = None

        # Derived state from odom
        self.prev_speed = None
        self.prev_time: Optional[Time] = None
        self.odom_speed = None
        self.odom_accel = None
        self.pose_xyz = (None, None, None)
        self.pose_yaw = None

        # CSV setup
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'sim_time_sec',
            'carma_speed_mps', 'carma_steering_rad', 'carma_accel_mps2', 'carma_jerk_mps3',
            'carla_throttle', 'carla_steer', 'carla_brake', 'carla_reverse',
            'carla_hand_brake', 'carla_manual_gear_shift', 'carla_gear',
            'actual_speed_mps', 'actual_accel_mps2', 'actual_gear',
            'pose_x', 'pose_y', 'pose_z', 'yaw_rad'
        ])
        self.rows_since_flush = 0

        self.get_logger().info(
            f"role_name='{role}'\n"
            f"  CARMA Ackermann:   {carma_ackermann_topic}\n"
            f"  CARLA Control:     {carla_control_topic}\n"
            f"  CARLA Status:      {self.carla_status_topic}\n"
            f"  CARLA Odometry:    {self.odom_topic}\n"
            f"  CSV:               {self.csv_path}"
        )

        # Clean exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _get_or_default_str(self, name: str, default: str) -> str:
        val = self.get_parameter(name).get_parameter_value().string_value
        return val if val else default

    # --- Sub callbacks ---
    def on_carma_ackermann(self, msg: AckermannDrive):
        self.last_carma = msg
        self._maybe_write_row('carma')

    def on_carla_control(self, msg: CarlaEgoVehicleControl):
        self.last_carla = msg
        self._maybe_write_row('carla')

    def on_carla_status(self, msg: CarlaEgoVehicleStatus):
        self.last_status = msg
        self._maybe_write_row('status')

    def on_odom(self, msg: Odometry):
        self.last_odom = msg
        p = msg.pose.pose.position
        self.pose_xyz = (p.x, p.y, p.z)
        self.pose_yaw = quat_to_yaw(msg.pose.pose.orientation)

        vx, vy, vz = msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        self.odom_speed = speed

        now = self.get_clock().now()
        if self.prev_speed is not None and self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 1e-6:
                self.odom_accel = (speed - self.prev_speed) / dt
        self.prev_speed = speed
        self.prev_time = now

        self._maybe_write_row('odom')

    def _maybe_write_row(self, _trigger: str):
        # Only log once we have some "actuals" context
        if self.last_status is None and self.last_odom is None:
            return
    
        sim_sec = self.get_clock().now().nanoseconds / 1e9
    
        # --- CARMA Ackermann (AckermannDrive, unstamped) ---
        carma_speed = carma_steer = carma_accel = carma_jerk = None
        if self.last_carma:
            carma_speed = self.last_carma.speed
            carma_steer = self.last_carma.steering_angle
            carma_accel = self.last_carma.acceleration
            carma_jerk  = self.last_carma.jerk
    
        # --- CARLA control command (CarlaEgoVehicleControl) ---
        carla_throttle = carla_steer = carla_brake = None
        carla_reverse = carla_hand_brake = carla_manual_gear_shift = None
        carla_gear = None
        if self.last_carla:
            c = self.last_carla
            carla_throttle = c.throttle
            carla_steer    = c.steer
            carla_brake    = c.brake
            carla_reverse  = bool(c.reverse)
            carla_hand_brake = bool(c.hand_brake)
            carla_manual_gear_shift = bool(c.manual_gear_shift)
            carla_gear = c.gear
    
        # --- Actuals (prefer CARLA status; fall back to odom) ---
        actual_speed = actual_accel = actual_gear = None
        if self.last_status:
            s = self.last_status
            # Speed (m/s)
            try:
                actual_speed = float(s.velocity)
            except Exception:
                actual_speed = None
    
            # Acceleration magnitude from geometry_msgs/Accel
            try:
                ax = float(s.acceleration.linear.x)
                ay = float(s.acceleration.linear.y)
                az = float(s.acceleration.linear.z)
                # Signed longitudinal accel: project onto vehicle forward
                # Use yaw from odom (vehicle heading in world/ENU)
                yaw = self.pose_yaw
                if yaw is not None:
                    fx, fy = math.cos(yaw), math.sin(yaw)   # forward unit vector
                actual_accel = ax*fx + ay*fy       # signed (m/s^2)
            except Exception:
                actual_accel = None
    
            # Gear (depending on bridge variant)
            try:
                if hasattr(s, 'control') and hasattr(s.control, 'gear'):
                    actual_gear = int(s.control.gear)
                elif hasattr(s, 'gear'):
                    actual_gear = int(s.gear)
            except Exception:
                actual_gear = None
    
        # Fallbacks from odom
        if actual_speed is None and self.odom_speed is not None:
            actual_speed = self.odom_speed
        if actual_accel is None and self.odom_accel is not None:
            actual_accel = self.odom_accel
    
        # Pose/yaw (from odom, since status lacks pose)
        x, y, z = self.pose_xyz
        yaw = self.pose_yaw
    
        # --- Write CSV row ---
        self.csv_writer.writerow([
            f"{sim_sec:.6f}",
            fmt(carma_speed), fmt(carma_steer), fmt(carma_accel), fmt(carma_jerk),
            fmt(carla_throttle), fmt(carla_steer), fmt(carla_brake),
            int(carla_reverse) if carla_reverse is not None else "",
            int(carla_hand_brake) if carla_hand_brake is not None else "",
            int(carla_manual_gear_shift) if carla_manual_gear_shift is not None else "",
            carla_gear if carla_gear is not None else "",
            fmt(actual_speed), fmt(actual_accel), actual_gear if actual_gear is not None else "",
            fmt(x), fmt(y), fmt(z), fmt(yaw),
        ])
    
        self.rows_since_flush += 1
        if self.rows_since_flush >= self.flush_every:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.rows_since_flush = 0


    # --- Shutdown ---
    def _signal_handler(self, *_):
        self.get_logger().info("Shutting down logger...")
        self._close_csv()
        rclpy.shutdown()

    def destroy_node(self):
        self._close_csv()
        super().destroy_node()

    def _close_csv(self):
        try:
            if not self.csv_file.closed:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.csv_file.close()
        except Exception as e:
            self.get_logger().warn(f"CSV close error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CarmaCarlaControlLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
