#!/usr/bin/env python
# Copyright (C) 2026 LEIDOS.
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
Subscribe from CARMA :carma_planning_msgs::RouteEvent
    Topic: /guidance/route_event
    ROUTE_COMPLETED resets the vehicle before triggering one reset signal. Duplicate completion events are
    ignored until ROUTE_STARTED rearms the node for the next route.

Publish :std_msgs::Empty
    Topic: /carla_loop/cycle_reset
    The message itself signals completion; Empty has no boolean data field.
    Route and guidance subscribers use this signal to restart their work.

Service :std_srvs::Trigger
    Service: /carla_loop/reset_vehicle
    Manually triggers the same reset at any time: aborts the active CARMA route
    (/guidance/abort_active_route), resets the vehicle to spawn_point and publishes
    /carla_loop/cycle_reset. Each call consumes one demo restart.
    e.g. ros2 service call /carla_loop/reset_vehicle std_srvs/srv/Trigger

Launch behavior:
    The bridge launch file starts this node only when demo_restart_count != 0.
    This node and the route and guidance subscribers enforce the restart limit.
"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from carma_planning_msgs.msg import RouteEvent
from carma_planning_msgs.srv import AbortActiveRoute
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


class CarmaCarlaCycleReset(Node):
    def __init__(self):
        super().__init__('cycle_reset', namespace='/carla_loop')

        self.restarts_remaining = self.declare_parameter('demo_restart_count', 0).value
        self.get_logger().info(
            f'Cycle reset parameter: demo_restart_count='
            f'{self.get_parameter("demo_restart_count").value}; '
            f'initialized restarts_remaining={self.restarts_remaining}')
        self.reset_vehicle = self.declare_parameter('reset_vehicle_on_loop', True).value
        self.host = self.declare_parameter('host', 'localhost').value
        self.port = self.declare_parameter('port', 2000).value
        self.timeout = self.declare_parameter('timeout', 10.0).value
        self.role_name = self.declare_parameter('role_name', 'carma_1').value
        self.spawn_point = self.declare_parameter('spawn_point', 'None').value

        # Internal state: allow the first completion even if route start was missed.
        self.completion_published = False
        self.completion_counted = False
        self.completed_runs = 0

        # Publisher
        self.reset_pub = self.create_publisher(Empty, '/carla_loop/cycle_reset', 10)

        # Subscription
        self.route_event_sub = self.create_subscription(
            RouteEvent, '/guidance/route_event', self.route_event_callback, 10
        )

        # Manual reset service; reentrant so it can wait on the abort route client
        reset_srv_group = ReentrantCallbackGroup()
        self.abort_route_client = self.create_client(
            AbortActiveRoute, '/guidance/abort_active_route', callback_group=reset_srv_group)
        self.reset_srv = self.create_service(
            Trigger, '/carla_loop/reset_vehicle', self.reset_service_callback,
            callback_group=reset_srv_group)
        self.get_logger().info(
            f'Cycle reset ready: listening on /guidance/route_event; '
            f'ROUTE_COMPLETED={RouteEvent.ROUTE_COMPLETED}; '
            f'demo_restart_count={self.restarts_remaining}; '
            f'reset_vehicle_on_loop={self.reset_vehicle}; '
            f'CARLA={self.host}:{self.port}; role_name={self.role_name!r}; '
            f'spawn_point={self.spawn_point!r}')

    def route_event_callback(self, msg):
        # Rearm when the next route starts.
        if msg.event == RouteEvent.ROUTE_STARTED:
            self.completion_published = False
            self.completion_counted = False
            self.get_logger().info('ROUTE_STARTED received; cycle reset rearmed.')
            return
        if msg.event != RouteEvent.ROUTE_COMPLETED:
            return

        self.get_logger().info('ROUTE_COMPLETED received on /guidance/route_event.')
        if self.completion_published:
            self.get_logger().info('Duplicate ROUTE_COMPLETED ignored; waiting for ROUTE_STARTED.')
            return

        # Count the finished route even when no restart remains or reset fails.
        if not self.completion_counted:
            self.completed_runs += 1
            self.completion_counted = True
            self.get_logger().info(f'Completed runs so far: {self.completed_runs}')
        if self.restarts_remaining == 0:
            self.get_logger().info('Vehicle reset skipped: no demo restarts remain.')
            return
        if self.reset_and_publish(self.reset_vehicle):
            self.get_logger().info('Route completed; published /carla_loop/cycle_reset.')

    def reset_service_callback(self, request, response):
        """Manually reset the vehicle to spawn_point and restart the route and guidance."""
        self.get_logger().info('Manual reset requested on /carla_loop/reset_vehicle.')
        if self.restarts_remaining == 0:
            response.success = False
            response.message = 'No demo restarts remain; route and guidance would not restart.'
            self.get_logger().warn(response.message)
            return response
        self.abort_active_route()
        response.success = self.reset_and_publish(True)
        response.message = ('Vehicle reset to spawn point; published /carla_loop/cycle_reset.'
                            if response.success else 'Vehicle reset failed; see node log.')
        self.get_logger().info(response.message)
        return response

    def abort_active_route(self, timeout_sec=5.0):
        """Abort the active CARMA route so the route can be set again after the reset."""
        if not self.abort_route_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('/guidance/abort_active_route unavailable; continuing reset.')
            return
        future = self.abort_route_client.call_async(AbortActiveRoute.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            self.get_logger().warn('Abort active route did not respond; continuing reset.')
        elif future.result().error_status == AbortActiveRoute.Response.NO_ACTIVE_ROUTE:
            self.get_logger().info('No active route to abort.')
        else:
            self.get_logger().info('Active route aborted.')

    def reset_and_publish(self, reset_vehicle):
        """Reset the vehicle (optional), publish one cycle reset and consume one restart."""
        if reset_vehicle:
            self.get_logger().info('Resetting actual CARLA vehicle to spawn point...')
            try:
                self.reset_vehicle_to_spawn()
            except (ImportError, RuntimeError, ValueError, IndexError) as error:
                self.get_logger().error(
                    f'Cannot reset vehicle; loop restart withheld: {error}')
                return False
        else:
            self.get_logger().info('Vehicle reset skipped: reset_vehicle_on_loop=false.')
        # Publish once per route and suppress repeated completion events until ROUTE_STARTED.
        self.reset_pub.publish(Empty())
        self.completion_published = True
        self.completion_counted = True
        if self.restarts_remaining > 0:
            self.restarts_remaining -= 1
        return True

    def reset_vehicle_to_spawn(self):
        """Reset the existing actor, preserving its attached sensors.

        Use CARLA coordinates and the same default map spawn as VehicleSpawner.
        Submit an ordered batch without ticking: the bridge owns the world tick.
        """
        import carla

        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        world = client.get_world()
        vehicles = [actor for actor in world.get_actors().filter('vehicle.*')
                    if actor.attributes.get('role_name') == self.role_name]
        if len(vehicles) != 1:
            raise RuntimeError(
                f'Expected one vehicle with role_name={self.role_name!r}, found {len(vehicles)}')
        vehicle = vehicles[0]
        if self.spawn_point.strip() and self.spawn_point != 'None':
            values = [float(value) for value in self.spawn_point.split(',')]
            if len(values) != 6 or not all(math.isfinite(value) for value in values):
                raise ValueError('spawn_point must contain finite x,y,z,roll,pitch,yaw values')
            transform = carla.Transform(
                carla.Location(x=values[0], y=values[1], z=values[2]),
                carla.Rotation(roll=values[3], pitch=values[4], yaw=values[5]))
        else:
            transform = world.get_map().get_spawn_points()[0]

        responses = client.apply_batch_sync([
            carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl()),
            carla.command.ApplyTransform(vehicle.id, transform),
            carla.command.ApplyTargetVelocity(vehicle.id, carla.Vector3D()),
            carla.command.ApplyTargetAngularVelocity(vehicle.id, carla.Vector3D()),
        ], False)
        errors = [response.error for response in responses if response.has_error()]
        if errors:
            raise RuntimeError('; '.join(errors))
        location = transform.location
        rotation = transform.rotation
        self.get_logger().info(
            f'CARLA reset SUCCESS: vehicle {self.role_name!r} (id={vehicle.id}) '
            f'reset commands accepted for spawn position '
            f'x={location.x:.3f}, y={location.y:.3f}, z={location.z:.3f}; '
            f'roll={rotation.roll:.3f}, pitch={rotation.pitch:.3f}, '
            f'yaw={rotation.yaw:.3f}; linear and angular velocity cleared.')


def main(args=None):
    rclpy.init(args=args)
    node = CarmaCarlaCycleReset()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
