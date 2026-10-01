#!/usr/bin/env python
# Copyright (C) 2023 LEIDOS.
# Migrated to ROS2 under Ryan Fleming @ UGA MSC Lab 2025
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
Subscribe from CARMA :geometry_msgs::PoseStamped
    Topic: /localization/current_pose

Subscribe :std_msgs::Empty (demo_restart_count != 0 only)
    Topic: /carla_loop/cycle_reset
    While restarts remain, the node idles after the route is set instead of exiting;
    each message on this topic restarts the route selection. demo_restart_count > 0
    limits the number of restarts, < 0 restarts indefinitely.

Call Services from CARMA:
    Service: /guidance/get_available_routes
             /guidance/set_active_route
"""
import rclpy
from rclpy.node import Node
import traceback

from geometry_msgs.msg import PoseStamped
from carma_planning_msgs.srv import GetAvailableRoutes, SetActiveRoute
from std_msgs.msg import Bool, Empty


class CarmaCarlaRoute(Node):

    def __init__(self):
        super().__init__('carma_carla_route')

        # Internal state
        self.reset_state()

        # Declare and get selected_route parameter
        self.selected_route_id = self.declare_parameter('selected_route', '').get_parameter_value().string_value
        if not self.selected_route_id:
            self.get_logger().error("No input route found. Please check config at /opt/carma/simulation/vehicle_config.json.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Selected route: {self.selected_route_id}")

        # Number of times to restart route selection on /carla_loop/cycle_reset before exiting.
        # 0 exits after the first route set, negative restarts indefinitely.
        self.restarts_remaining = self.declare_parameter('demo_restart_count', 0).get_parameter_value().integer_value
        self.get_logger().info(f"Demo restart count: {self.restarts_remaining}")

        # Subscriptions
        self.create_subscription(PoseStamped, '/localization/current_pose', self.pose_cb, 10)
        self.create_subscription(Bool, '/carla/guidance_bridge_node/active_status', self.guidance_bridge_node_status_callback, 10)
        if self.restarts_remaining != 0:
            self.create_subscription(Empty, '/carla_loop/cycle_reset', self.cycle_reset_cb, 10)

        # Timer-based loop (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def reset_state(self):
        self.received_pose = False
        self.is_guidance_bridge_node_active = False
        self.route_set = False
        self.get_available_routes_future = None
        self.set_route_future = None
        self.available_route_ids = []

    def cycle_reset_cb(self, msg):
        if self.restarts_remaining == 0:
            self.get_logger().info("Received /carla_loop/cycle_reset, but no restarts remain; ignoring.")
            return
        if self.restarts_remaining > 0:
            self.restarts_remaining -= 1
        self.get_logger().info(
            f"Received /carla_loop/cycle_reset, restarting route selection (restarts remaining: {self.restarts_remaining}).")
        self.reset_state()
        # Restarts the timer if it was cancelled after the previous route set
        self.timer.reset()

    def pose_cb(self, msg):
        self.received_pose = True

    def guidance_bridge_node_status_callback(self, msg):
        self.is_guidance_bridge_node_active = msg.data

    def timer_cb(self):
        if self.route_set:
            if self.restarts_remaining != 0:
                # Idle until the next /carla_loop/cycle_reset
                self.get_logger().info("Route set, waiting for /carla_loop/cycle_reset.")
                self.timer.cancel()
                return
            # Clean shutdown
            self.get_logger().info("Shutting down after successful route set.")
            self.destroy_node()
            rclpy.shutdown()
            return

        if not self.received_pose:
            self.get_logger().info("Waiting for /localization/current_pose...")
            return

        if not self.is_guidance_bridge_node_active:
            self.get_logger().warn("Guidance bridge not yet active.")
            return

        if not self.get_available_routes_future:
            self.get_logger().info("Requesting available routes...")
            client = self.create_client(GetAvailableRoutes, '/guidance/get_available_routes')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("/guidance/get_available_routes service not available.")
                return
            request = GetAvailableRoutes.Request()
            self.get_available_routes_future = client.call_async(request)
            self.get_available_routes_future.add_done_callback(self.get_available_routes_cb)
            return
        elif not self.get_available_routes_future.done():
            self.get_logger().info("Waiting for GetAvailableRoutes service call to finish...")
            return

        if not self.set_route_future:
            self.get_logger().info(f"Sending SetActiveRoute for route: {self.selected_route_id}")
            client = self.create_client(SetActiveRoute, '/guidance/set_active_route')
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("/guidance/set_active_route service not available.")
                return
            request = SetActiveRoute.Request()
            request.choice = 0
            request.route_id = self.selected_route_id
            self.set_route_future = client.call_async(request)
            self.set_route_future.add_done_callback(self.set_route_cb)
        if not self.set_route_future.done():
            self.get_logger().info("Waiting for SetActiveRoute service call to finish")

    def get_available_routes_cb(self, future):
        # Ignore responses to requests made before a cycle reset
        if future is not self.get_available_routes_future:
            return
        try:
            result = future.result()
            self.available_route_ids = [r.route_id for r in result.available_routes]
            self.get_logger().info(f"Available routes: {self.available_route_ids}")
        except Exception as e:
            self.get_logger().error("Service call to /guidance/get_available_routes failed.")
        finally:
            if self.selected_route_id in self.available_route_ids:
                self.get_logger().info(f"Selected route {self.selected_route_id} is valid.")
            else:
                self.get_logger().warn(f"Selected route {self.selected_route_id} not found in available routes.")
                self.get_logger().warn("Retrying route query...")
                self.get_available_routes_future = None

    def set_route_cb(self, future):
        # Ignore responses to requests made before a cycle reset
        if future is not self.set_route_future:
            return
        try:
            result = future.result()
            if result and not result.error_status:
                self.get_logger().info("Call to set_active_route succeeded!")
                self.route_set = True
            else:
                self.get_logger().error("SetActiveRoute failed: Error status or null result.")
        except Exception as e:
            self.get_logger().error(f"Exception during SetActiveRoute: {e}")
        finally:
            self.set_route_future = None


def main(args=None):
    rclpy.init(args=args)
    print("carma_carla_route")
    node = None
    try:
        node = CarmaCarlaRoute()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        logger = rclpy.logging.get_logger("carma_carla_route_main")
        if node:
            logger = node.get_logger()
        logger.error(f"Unhandled exception in {node.get_name() if node else 'CarmaCarlaRoute'}: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()