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
Converts objects detected by lidar sensor from carla-sensor-lib to CARMA ExternalObjectList messages.
Subscribes to: None
Publishes: /environment/external_objects (cav_msgs/ExternalObjectList) 
"""

import sys

sys.path.append('/home/carma/carla-sensor-lib')
from src.CarlaCDASimAPI import CarlaCDASimAPI
from src.util.SimulatedSensorUtils import SimulatedSensorUtils
import carla

from carma_perception_msgs.msg import ExternalObjectList, ExternalObject
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.exceptions import ROSInterruptException

from scipy.spatial.transform import Rotation
import traceback

class CarlaToCarmaSensorExternalObjectsNode(Node):
    def __init__(self):
        super().__init__('carla_to_carma_sensor_external_objects')

        # Parameter declarations with logging
        self.declare_parameter('sensor_object_pub_rate', 10)
        self.pub_rate = self.get_parameter('sensor_object_pub_rate').value
        self.get_logger().info(f"Parameter [sensor_object_pub_rate]: {self.pub_rate}")

        self.sensor_creation_rate = self.create_rate(1.0)

        # CARLA setup
        self.get_logger().info("Initializing CARLA client and retrieving configuration...")
        setup_result = self.setup_carla_client_and_config()
        if setup_result is None:
            self.get_logger().fatal("CARLA setup failed — sensor cannot be created.")
            return None

        self.get_logger().info("Creating /environment/external_objects publisher...")
        self.external_objects_pub = self.create_publisher(ExternalObjectList, '/environment/external_objects', 1)
        self.timer = self.create_timer(1.0 / self.pub_rate, self.timer_callback)

        client, world, api, parent_actor_id, sensor_config, noise_model_config = setup_result

        # Sensor creation loop with logging
        sensor_creation_start_time = self.get_clock().now()
        sensor_creation_timeout = 60
        self.sensor = None
        while rclpy.ok() and self.sensor is None:
            self.get_logger().info("Attempting to create simulated semantic lidar sensor...")
            self.sensor = self.create_sensor_for_sensorlib(world, api, parent_actor_id, sensor_config, noise_model_config)

            if self.sensor:
                self.get_logger().info("Sensor created successfully.")
                break
            elif (self.get_clock().now() - sensor_creation_start_time) > Duration(seconds=sensor_creation_timeout):
                self.get_logger().error(f"Sensor creation timed out after {sensor_creation_timeout} seconds.")
                return
            else:
                self.get_logger().warn("Sensor creation failed; retrying shortly...")
                self.sensor_creation_rate.sleep()

    def setup_carla_client_and_config(self):
        # Declare + log parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('role_name', 'hero')

        carla_host_address = self.get_parameter('host').value
        carla_port = self.get_parameter('port').value
        self.role_name = self.get_parameter('role_name').value

        self.get_logger().info(f"CARLA host: {carla_host_address}, port: {carla_port}, role_name: {self.role_name}")

        # Connect to CARLA
        try:
            client = carla.Client(carla_host_address, carla_port)
            client.set_timeout(10.0)
            world = client.get_world()
            self.get_logger().info("Successfully connected to CARLA and retrieved world.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return None

        api = CarlaCDASimAPI.build_from_world(world)

        # Load sensor config
        self.get_logger().info("Loading sensor and noise model config files...")
        sensor_config = SimulatedSensorUtils.load_config_from_file(
            "/home/carma/carla-sensor-lib/config/simulated_sensor_config.yaml"
        )
        noise_model_config = SimulatedSensorUtils.load_config_from_file(
            "/home/carma/carla-sensor-lib/config/noise_model_config.yaml"
        )
        sensor_config["simulated_sensor"]["use_sensor_centric_frame"] = False

        # Search for actor with matching role_name
        self.get_logger().info(f"Searching for vehicle with role_name = '{self.role_name}'")
        parent_actor_id = None
        actor_search_timeout = 15
        search_start_time = self.get_clock().now()

        while (self.get_clock().now() - search_start_time) < Duration(seconds=actor_search_timeout):
            for actor in world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    parent_actor_id = actor.id
                    self.get_logger().info(f"Found actor with ID {parent_actor_id} for role_name '{self.role_name}'")
                    break
            if parent_actor_id:
                break
            if not rclpy.ok():
                raise ROSInterruptException("ROS shutdown requested while searching for actor.")
            self.get_logger().warn("Vehicle not found yet; retrying...")
            time.sleep(1)

        if not parent_actor_id:
            self.get_logger().error(
                f"Timed out after {actor_search_timeout} seconds searching for actor with role_name '{self.role_name}'."
            )
            return None

        return client, world, api, parent_actor_id, sensor_config, noise_model_config

    def create_sensor_for_sensorlib(self, world, api, parent_actor_id, sensor_config, noise_model_config):
        # Log sensor parameters
        self.declare_parameter('sensor_id', 'sim_lidar_1')
        self.declare_parameter('detection_cycle_delay_seconds', 1.0)

        sensor_id = self.get_parameter('sensor_id').value
        delay = self.get_parameter('detection_cycle_delay_seconds').value

        self.get_logger().info(f"Creating sensor with ID '{sensor_id}', delay {delay}s")

        user_offset = carla.Location(0, 0, 2.0)
        lidar_transform = carla.Transform(user_offset)
        infrastructure_id = "1"

        sensor = api.create_simulated_semantic_lidar_sensor(
            sensor_config["simulated_sensor"],
            sensor_config["lidar_sensor"],
            noise_model_config,
            delay,
            infrastructure_id,
            sensor_id,
            lidar_transform.location,
            lidar_transform.rotation,
            parent_actor_id
        )
        return sensor
    
    def publish(self, object_msg_list):
        self.external_objects_pub.publish(object_msg_list)

    def timer_callback(self):
        if not rclpy.ok():
            return
        if self.sensor is not None:
            get_data_from_sensorlib(self.sensor, self)

def get_data_from_sensorlib(sensor, node):

    results = sensor.get_detected_objects()
    object_msg_list = ExternalObjectList()
    object_msg_list.header.frame_id = "map"

    for object in results:
        object_msg = ExternalObject()
        object_msg.header.stamp = node.get_clock().now().to_msg()
        object_msg.header.frame_id = "map"

        object_msg.id = object.objectId
        position_list = object.position.tolist()
        object_msg.pose.pose.position.x = float(position_list[0])
        object_msg.pose.pose.position.y = float(position_list[1])
        object_msg.pose.pose.position.z = 0.0 #carma-platform doesn't use z value

        roll, pitch, yaw = object.rotation
        rotation = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
        quaternion = rotation.as_quat()

        object_msg.pose.pose.orientation.x = quaternion[0]
        object_msg.pose.pose.orientation.y = quaternion[1]
        object_msg.pose.pose.orientation.z = quaternion[2]
        object_msg.pose.pose.orientation.w = quaternion[3]

        zeros_3_3 = np.zeros((3,3),dtype=float) # Create off-diagonal zeros array
        pose_covariance = np.asarray(np.bmat([[object.positionCovariance, zeros_3_3], [zeros_3_3, object.orientationCovariance]]))

        object_msg.pose.covariance = pose_covariance.flatten()

        object_msg.confidence = object.confidence

        velocity_list = object.velocity.tolist()
        object_msg.velocity.twist.linear.x = float(velocity_list[0])
        object_msg.velocity.twist.linear.y = float(velocity_list[1])
        object_msg.velocity.twist.linear.z = float(velocity_list[2])

        twist_covariance = np.asarray(np.bmat([[object.velocityCovariance, zeros_3_3], [zeros_3_3, object.angularVelocityCovariance]]))
        object_msg.velocity.covariance = twist_covariance.flatten()

        object_msg.size.x = object.size[0]
        object_msg.size.y = object.size[1]
        object_msg.size.z = object.size[2]

        #Since SDSM service is not populating the optional data about object_type
        #treating every vehicle as CAR will improve the performance of CP algorithm
        #https://github.com/usdot-fhwa-stol/carma-streets/issues/402

        if object.type == "PEDESTRIAN":
            object_msg.object_type = 4
        elif object.type == "MOTORCYCLE" or object.type == "CYCLIST":
            object_msg.object_type = 3
        elif object.type == "TRUCK" or object.type == "VAN" or object.type == "CAR":
            object_msg.object_type = 1
        else:
            object_msg.object_type = 0

        object_msg.dynamic_obj = True

        object_msg.presence_vector = object_msg.presence_vector + \
                                 ExternalObject.ID_PRESENCE_VECTOR + \
                                 ExternalObject.POSE_PRESENCE_VECTOR + \
                                 ExternalObject.VELOCITY_PRESENCE_VECTOR + \
                                 ExternalObject.SIZE_PRESENCE_VECTOR + \
                                 ExternalObject.CONFIDENCE_PRESENCE_VECTOR + \
                                 ExternalObject.DYNAMIC_OBJ_PRESENCE + \
                                 ExternalObject.OBJECT_TYPE_PRESENCE_VECTOR
        object_msg_list.objects.append(object_msg)

    node.publish(object_msg_list)

def main(args=None):
    rclpy.init(args=args)
    print("carla_to_carma_sensor_external_objects")
    try:
        node = CarlaToCarmaSensorExternalObjectsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        # Ensure logger is available or use print
        logger = rclpy.logging.get_logger("carla_to_carma_sensor_external_objects_node_main")
        if node : logger = node.get_logger()
        logger.error(f"Unhandled exception in {node.get_name() if node else 'CarlaToCarmaObjectsNode'}: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
