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
Subscribe from CARMA :cav_msgs::RouteEvent
    Topic: /guidance/route_event

Call Services from CARMA:
    Service: /guidance/set_guidance_active
             /guidance/plugins/get_active_plugins
"""
import rclpy
from rclpy.node import Node

import ast
import traceback

from cav_msgs.msg import RouteEvent
from cav_msgs.msg import Plugin
from cav_srvs.srv import SetGuidanceActive, PluginList
from std_msgs.msg import Bool

class CarmaCarlaGuidance(Node):
    def __init__(self):
        super().__init__('carma_carla_guidance')

        # Get Parameters
        self.declare_parameter("selected_plugins", "[]")
        self.declare_parameter("start_delay_in_seconds", 15.0)
        plugin_list_str = self.get_parameter("selected_plugins").get_parameter_value().string_value
        self.selected_plugin_list = ast.literal_eval(plugin_list_str)
        self.start_delay = self.get_parameter("start_delay_in_seconds").get_parameter_value().double_value

        if len(self.selected_plugin_list) == 0 or not self.selected_plugin_list:
            self.get_logger().error(
                "No input plugin found. Check the config file at /opt/carma/simulation/vehicle_config.json"
            )
            return
        
         # route selected state
        self.route_selected = False

        # Publishers/Subscribers
        self.status_pub = self.create_publisher(Bool, '/carla/guidance_bridge_node/active_status', 10)
        self.create_subscription(RouteEvent, '/guidance/route_event', self.route_event_callback, 10)

        # CARMA Services
        self.set_guidance_active_client = self.create_client(SetGuidanceActive, '/guidance/set_guidance_active')
        self.get_active_plugins_client = self.create_client(PluginList, '/guidance/plugins/get_active_plugins')

        # futures for aysnc CARMA service calls
        self.plugin_list_future = None
        self.engage_guidance_future = None
        self.attempting_engage = False
        self.engage_delay_waited_seconds = 0

        # store results of service calls
        self.active_plugins = []
        self.guidance_status = False
        self.selected_plugins_active = False

        # Logging
        self.get_logger().info(f"Start delay in seconds: {self.start_delay}")
        self.get_logger().info(f"List of plugins from config: {self.selected_plugin_list}")

        # Wait for required services
        self.get_logger().info('Waiting for services...')
        while not self.get_active_plugins_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for get_active_plugins service...")
        while not self.set_guidance_active_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for set_guidance_active service...')

        # create timer
        self.create_timer(1.0, self.timer_cb)
    
    def timer_cb(self):
        # publish node active status
        status_msg = Bool()
        status_msg.data = True
        self.status_pub.publish(status_msg)

        if self.guidance_status:
            self.destroy_node()
            rclpy.shutdown()

        if not self.plugin_list_future:
            try:
                request = PluginList.Request()
                self.plugin_list_future = self.get_active_plugins_client.call_async(request)
                self.plugin_list_future.add_done_callback(self.pluginlist_cb)
                return
            except Exception as e:
                self.get_logger().warn(f"Failed to send PluginList request: {e}")
                return
        elif not self.plugin_list_future.done():
            self.get_logger().info("Waiting for PluginList service response...")
            return

        if not self.route_selected:
            self.get_logger().warn("Could not engage guidance: route not selected")
            return
        elif not self.selected_plugins_active:
            active_names = [plugin.name for plugin in self.active_plugins]
            self.get_logger().warn("Could not engage guidance: missing some required plugins")
            self.get_logger().warn(f"Active plugins: {active_names}")
            self.plugin_list_future = None
        else:
            if not self.attempting_engage:
                if self.engage_delay_waited_seconds >= self.start_delay:
                    self.attempting_engage = True
                    return
                self.get_logger().info(f"Engaging the guidance in: {self.start_delay - self.engage_delay_waited_seconds:.0f}")
                self.engage_delay_waited_seconds += 1.0
                return
            else:
                if not self.engage_guidance_future:
                    request = SetGuidanceActive.Request()
                    request.guidance_active = True
                    self.engage_guidance_future = self.set_guidance_active_client.call_async(request)
                    self.engage_guidance_future.add_done_callback(self.engage_guidance_cb)
                    return
                elif not self.engage_guidance_future.done():
                    self.get_logger().info("Waiting for SetEngageGuidance service response...")
    
    def pluginlist_cb(self, future):
        try:
            response = future.result()
            self.active_plugins = response.plugins
            self.get_logger().info("Received active plugins")
            self.selected_plugins_active = self.check_plugin_status(self.active_plugins)
        except Exception as e:
                self.get_logger().warn(f"Service call to {self.get_active_plugins_client.srv_name} failed: {e}")
                self.get_logger().warn("Service call can sometimes fail due to ROS, but please make sure the selected plugins are activated. Retrying in 1 second..")
                self.active_plugins = []
                self.plugin_list_future = None
    
    def engage_guidance_cb(self, future):
        try:
            result = future.result()
            if result.guidance_status:
                self.get_logger().info("Guidance engaged")
                self.guidance_status = True
            else:
                self.get_logger().error("Guidance engagement failed, retrying...")
        except Exception as e:
            self.get_logger().warn(f"Service call to {self.set_guidance_active_client.srv_name} failed: {e}")
            self.get_logger().warn("Service call can sometimes fail due to ROS, but please make sure the platform has started without any error. Retrying in 1 second..")  
        finally:
            self.attempting_engage = False
            self.engage_delay_waited_seconds = 0.0
            self.engage_guidance_future = None

    def check_plugin_status(self, active_plugins):
        """
        check active plugin with plugin list
        plugin_list: selected plugins + required plugins
        activate_plugins: the PluginList in /guidance/plugins/get_active_plugins ROS service
        """
        active_names = [p.name for p in active_plugins if p.available and p.activated]
        missing = list(set(self.selected_plugin_list) - set(active_names))
        if len(missing) > 0:
            self.get_logger().warn(f"Some plugins are not activated: {missing}")
            return False
        self.get_logger().info("All plugins activated...!")
        return True
    
    def route_event_callback(self, msg):
        if msg.event == RouteEvent.ROUTE_SELECTED or msg.event == RouteEvent.ROUTE_STARTED:
            self.route_selected = True

def main(args=None):
    rclpy.init(args=args)
    print("carma_carla_guidance")
    node = None
    try:
        node = CarmaCarlaGuidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        # Ensure logger is available or use print
        logger = rclpy.logging.get_logger("carma_carla_guidance_main")
        if node : logger = node.get_logger()
        logger.error(f"Unhandled exception in {node.get_name() if node else 'carma_carla_guidance'}: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
