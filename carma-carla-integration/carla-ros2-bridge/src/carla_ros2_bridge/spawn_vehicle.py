#!/usr/bin/env python3
# Copyright (C) 2021 LEIDOS.
# Developed by Ryan Fleming @ UGA MSC Lab 2025
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
#
#
# This file is loosely based on the reference architecture developed by the Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# https://https://github.com/carla-simulator/carla/blob/ue5-dev/PythonAPI/examples/ros2/ros2_native.py
#
# That file has the following license and some code snippets from it may be present in this file as well and are under the same license.
#
# Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
VehicleSpawner class which handles spawning vehicle described by config file 
at path specificed by parameter 'config_file'
"""
import json
import carla
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import logging


class VehicleSpawner(Node):

    def __init__(self):
        super().__init__('spawn_vehicle_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('autopilot', False)
        self.declare_parameter('spawn_point', '')

        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.use_autopilot = self.get_parameter('autopilot').get_parameter_value().bool_value
        self.spawn_point = self.get_parameter('spawn_point').get_parameter_value().string_value

        self.vehicle = None
        self.sensors = []
        self.world = None

        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.get_logger().info(f"[Spawner] Connected to CARLA at {self.host}:{self.port}")

            self.spawn_from_file(self.config_file)

            if self.use_autopilot and self.vehicle:
                self.vehicle.set_autopilot(True)
                self.get_logger().info(f"[Spawner] Autopilot enabled for vehicle '{self.vehicle.attributes.get('role_name')}'")

            self.get_logger().info("[Spawner] Vehicle and sensors spawned successfully.")
            # After spawning vehicle + sensors
            self.get_logger().info("[Spawner] Spawning complete. Forcing world tick to register actors.")
            self.world.tick()


        except Exception as e:
            self.get_logger().error(f"Failed to initialize spawner: {e}")
            self.destroy_node()
            raise

    def spawn_from_file(self, file_path):
        with open(file_path, 'r') as f:
            config = json.load(f)

        self.vehicle = self._spawn_vehicle(config)
        self.sensors = self._spawn_sensors(config.get("sensors", []), self.vehicle)

    def _spawn_vehicle(self, config):
        bp_library = self.world.get_blueprint_library()
        bp = bp_library.filter(config.get("type"))[0]
        bp.set_attribute("role_name", config.get("id"))
        bp.set_attribute("ros_name", config.get("id"))

        # Extract spawn point values
        spawn_point_vals = [float(num) for num in self.spawn_point.split(',')]
        # Create CARLA Transform from spawn_point struct
        if len(spawn_point_vals) == 6:
            transform = carla.Transform(
                location=carla.Location(x=spawn_point_vals[0], y=spawn_point_vals[1], z=spawn_point_vals[2]),
                rotation=carla.Rotation(roll=spawn_point_vals[3], pitch=spawn_point_vals[4], yaw=spawn_point_vals[5])
            )
        else:
            # Fallback to default spawn point if no spawn_point provided
            transform = self.world.get_map().get_spawn_points()[0]
            
        vehicle = self.world.spawn_actor(bp, transform)
        self.get_logger().info(f"[Spawner] Spawned vehicle '{config.get('id')}'")
        return vehicle

    def _spawn_sensors(self, sensors_config, vehicle):
        bp_library = self.world.get_blueprint_library()
        spawned_sensors = []

        for sensor in sensors_config:
            bp = bp_library.filter(sensor.get("type"))[0]
            bp.set_attribute("role_name", sensor["id"])
            bp.set_attribute("ros_name", sensor["id"])
            for key, value in sensor.get("attributes", {}).items():
                bp.set_attribute(str(key), str(value))

            sp = sensor["spawn_point"]
            transform = carla.Transform(
                location=carla.Location(x=sp["x"], y=-sp["y"], z=sp["z"]),
                rotation=carla.Rotation(roll=sp["roll"], pitch=-sp["pitch"], yaw=-sp["yaw"])
            )

            sensor_actor = self.world.spawn_actor(bp, transform, attach_to=vehicle)
            sensor_actor.enable_for_ros()
            spawned_sensors.append(sensor_actor)
            self.get_logger().info(f"[Spawner] Spawned sensor '{sensor.get('id')}'")

        return spawned_sensors

    def destroy(self):
        self.get_logger().info("Destroying spawned actors...")
        try:
            for sensor in self.sensors:
                sensor.destroy()
            if self.vehicle:
                self.vehicle.destroy()
        except Exception as e:
            self.get_logger().warn(f"Error during cleanup: {e}")
        finally:
            self.get_logger().info("Cleanup complete.")
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
