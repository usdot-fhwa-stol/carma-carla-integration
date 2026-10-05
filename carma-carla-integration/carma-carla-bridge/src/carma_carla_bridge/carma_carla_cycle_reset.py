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
    ROUTE_COMPLETED triggers one reset signal. Duplicate completion events are
    ignored until ROUTE_STARTED rearms the node for the next route.

Publish :std_msgs::Empty
    Topic: /carla_loop/cycle_reset
    The message itself signals completion; Empty has no boolean data field.
    Route and guidance subscribers use this signal to restart their work.

Launch behavior:
    The bridge launch file starts this node only when demo_restart_count != 0.
    This node keeps listening; route and guidance subscribers enforce their own
    restart limits.
"""

import rclpy
from rclpy.node import Node

from carma_planning_msgs.msg import RouteEvent
from std_msgs.msg import Empty


class CarmaCarlaCycleReset(Node):
    def __init__(self):
        super().__init__('cycle_reset', namespace='/carla_loop')

        # Internal state: allow the first completion even if route start was missed.
        self.completion_published = False

        # Publisher
        self.reset_pub = self.create_publisher(Empty, '/carla_loop/cycle_reset', 10)

        # Subscription
        self.route_event_sub = self.create_subscription(
            RouteEvent, '/guidance/route_event', self.route_event_callback, 10
        )

    def route_event_callback(self, msg):
        # Rearm when the next route starts.
        if msg.event == RouteEvent.ROUTE_STARTED:
            self.completion_published = False
        elif msg.event == RouteEvent.ROUTE_COMPLETED and not self.completion_published:
            # Publish once per route and suppress repeated completion events.
            self.reset_pub.publish(Empty())
            self.completion_published = True
            self.get_logger().info('Route completed; published /carla_loop/cycle_reset.')


def main(args=None):
    rclpy.init(args=args)
    node = CarmaCarlaCycleReset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
