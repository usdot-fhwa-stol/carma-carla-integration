#!/usr/bin/env python3
# Copyright (C) 2021 LEIDOS.
# Migrated to ROS2 under Will Varner @ UGA MSC Lab 2025
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
# This file is based on the original ROS1 version and has been migrated to ROS2.
# Original file's Intel Corporation license notice also applies to derived parts.
# Copyright (c) 2018-2019 Intel Corporation (for original derived parts)
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Converts CARLA native ROS2 detected objects to CARMA ExternalObjectList messages.
Subscribes to: /carla/<role_name>/objects (derived_object_msgs/ObjectArray)
Publishes: /environment/external_objects (cav_msgs/ExternalObjectList)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time as RclpyTime
import threading
import traceback

from derived_object_msgs.msg import ObjectArray as DerivedObjectArray
from derived_object_msgs.msg import Object as DerivedObject
from carma_perception_msgs.msg import ExternalObjectList, ExternalObject

class CarlaToCarmaObjectsNode(Node):
    """Node to convert CARLA detected objects to CARMA ExternalObjectList messages."""
    def __init__(self):
        super().__init__('carla_to_carma_external_objects_node')

        # Declare and get parameters
        self.declare_parameter('role_name', 'hero')
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.get_logger().info(f"Using role_name: '{self.role_name}'")

        # Store previous objects for velocity calculation and tracking
        self.prev_objects = {}
        self.prev_objects_lock = threading.Lock() # Initialize the lock for thread safety

        # Object type lookup based on derived_object_msgs/Object constants
        self.object_type_lookup = {
            DerivedObject.CLASSIFICATION_UNKNOWN: ExternalObject.UNKNOWN,
            DerivedObject.CLASSIFICATION_UNKNOWN_SMALL: ExternalObject.UNKNOWN,
            DerivedObject.CLASSIFICATION_UNKNOWN_MEDIUM: ExternalObject.UNKNOWN,
            DerivedObject.CLASSIFICATION_UNKNOWN_BIG: ExternalObject.UNKNOWN,
            DerivedObject.CLASSIFICATION_PEDESTRIAN: ExternalObject.PEDESTRIAN,
            DerivedObject.CLASSIFICATION_BIKE: ExternalObject.MOTORCYCLE, 
            DerivedObject.CLASSIFICATION_CAR: ExternalObject.SMALL_VEHICLE,
            DerivedObject.CLASSIFICATION_TRUCK: ExternalObject.LARGE_VEHICLE,
            DerivedObject.CLASSIFICATION_MOTORCYCLE: ExternalObject.MOTORCYCLE,
            DerivedObject.CLASSIFICATION_OTHER_VEHICLE: ExternalObject.UNKNOWN,
            DerivedObject.CLASSIFICATION_BARRIER: ExternalObject.UNKNOWN, 
            DerivedObject.CLASSIFICATION_SIGN: ExternalObject.UNKNOWN,
        }
        self.get_logger().info(f"Object type lookup table configured using derived_object_msgs constants.")

        # QoS Profile for publisher
        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5, # Keep a few messages in case of bursty subscribers
            durability=DurabilityPolicy.VOLATILE # Objects are dynamic, no need for latching
        )
        self.external_objects_pub = self.create_publisher(
            ExternalObjectList,
            '/environment/external_objects',
            qos_profile_pub)
        self.get_logger().info(f"Publishing ExternalObjectList on '/environment/external_objects'")

        # QoS Profile for subscriber
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Assume input should be reliable
            history=HistoryPolicy.KEEP_LAST,
            depth=10 # Allow a small buffer for incoming messages
        )
        carla_objects_topic = f'/carla/{self.role_name}/objects'
        self.carla_objects_sub = self.create_subscription(
            DerivedObjectArray,
            carla_objects_topic,
            self.carla_objects_callback,
            qos_profile_sub)
        self.get_logger().info(f"Subscribed to DerivedObjectArray on '{carla_objects_topic}'")

        # Timer for cleaning up stale objects
        self.cleanup_timer_period = 1.0  # seconds
        self.cleanup_timer = self.create_timer(self.cleanup_timer_period, self.cleanup_stale_objects)
        self.max_no_update_cycles = 10 # Number of cleanup cycles an object can be missed

    def carla_objects_callback(self, carla_obj_array_msg: DerivedObjectArray):
        """Callback for CARLA objects, converts and publishes as CARMA ExternalObjectList."""
        current_ros_time = self.get_clock().now() # RclpyTime object

        carma_objects_list_msg = ExternalObjectList()
        carma_objects_list_msg.header = carla_obj_array_msg.header # Propagate header

        active_ids_in_current_message = set()
        processed_carma_objects = []

        # Process objects and prepare data for prev_objects update
        # This part does not need the lock yet, as it's local to the callback
        for carla_obj in carla_obj_array_msg.objects:
            obj_id = carla_obj.id 
            active_ids_in_current_message.add(obj_id)

            # Pass obj_id to convert_carla_to_carma for velocity calculation context
            carma_obj_msg = self.convert_carla_to_carma(carla_obj, current_ros_time, obj_id)
            if carma_obj_msg:
                processed_carma_objects.append(carma_obj_msg)
        
        carma_objects_list_msg.objects = processed_carma_objects
        self.external_objects_pub.publish(carma_objects_list_msg)

        # Safely update prev_objects and no_update_counts
        with self.prev_objects_lock:
            # Add/Update currently seen objects
            for carma_obj_converted, carla_obj_original in zip(processed_carma_objects, carla_obj_array_msg.objects):
                original_carla_object = None
                for co in carla_obj_array_msg.objects:
                    if co.id == carma_obj_converted.id:
                        original_carla_object = co
                        break
                
                if original_carla_object:
                    self.prev_objects[carma_obj_converted.id] = {
                        'object_msg': carma_obj_converted, 
                        'raw_pose': original_carla_object.pose,   
                        'raw_header_stamp_ros': RclpyTime.from_msg(original_carla_object.header.stamp), 
                        'update_time_ros': current_ros_time, 
                        'no_update_count': 0 
                    }

            # Increment no_update_count for objects not in the current message
            for obj_id_key in self.prev_objects: 
                if obj_id_key not in active_ids_in_current_message:
                    if 'no_update_count' in self.prev_objects[obj_id_key]:
                        self.prev_objects[obj_id_key]['no_update_count'] += 1
                    else:
                        self.prev_objects[obj_id_key]['no_update_count'] = 1
                        self.get_logger().warn(f"Object ID {obj_id_key} was missing 'no_update_count', initialized to 1.")


    def convert_carla_to_carma(self, carla_obj: DerivedObject, current_ros_time: RclpyTime, obj_id_int: int) -> ExternalObject | None:
        """Converts a single CARLA DerivedObject to a CARMA ExternalObject."""
        try:
            carma_obj = ExternalObject()
            carma_obj.header = carla_obj.header # Propagate individual object header

            # Initialize presence vector
            carma_obj.presence_vector = (
                ExternalObject.ID_PRESENCE_VECTOR |
                ExternalObject.POSE_PRESENCE_VECTOR |
                ExternalObject.VELOCITY_PRESENCE_VECTOR |
                ExternalObject.SIZE_PRESENCE_VECTOR |
                ExternalObject.CONFIDENCE_PRESENCE_VECTOR |
                ExternalObject.DYNAMIC_OBJ_PRESENCE
            )

            carma_obj.id = obj_id_int # Use passed obj_id_int
            carma_obj.pose.pose = carla_obj.pose

            # Velocity
            if (abs(carla_obj.twist.linear.x) > 1e-3 or \
                abs(carla_obj.twist.linear.y) > 1e-3 or \
                abs(carla_obj.twist.linear.z) > 1e-3):
                carma_obj.velocity.twist = carla_obj.twist
            else: 
                carma_obj.velocity.twist.linear.x = 0.0
                carma_obj.velocity.twist.linear.y = 0.0
                carma_obj.velocity.twist.linear.z = 0.0
                carma_obj.velocity.twist.angular.x = 0.0
                carma_obj.velocity.twist.angular.y = 0.0
                carma_obj.velocity.twist.angular.z = 0.0

                with self.prev_objects_lock: # Access prev_objects safely
                    if obj_id_int in self.prev_objects and \
                       self.prev_objects[obj_id_int].get('no_update_count', -1) == 0 and \
                       'raw_pose' in self.prev_objects[obj_id_int]:
                        
                        prev_data = self.prev_objects[obj_id_int]
                        prev_pose = prev_data['raw_pose']
                        prev_stamp_ros = prev_data['raw_header_stamp_ros']
                        
                        current_obj_stamp_ros = RclpyTime.from_msg(carla_obj.header.stamp)
                        time_diff_secs = (current_obj_stamp_ros.nanoseconds - prev_stamp_ros.nanoseconds) / 1e9

                        if time_diff_secs > 1e-9: 
                            dx = carla_obj.pose.position.x - prev_pose.position.x
                            dy = carla_obj.pose.position.y - prev_pose.position.y
                            dz = carla_obj.pose.position.z - prev_pose.position.z
                            
                            carma_obj.velocity.twist.linear.x = dx / time_diff_secs
                            carma_obj.velocity.twist.linear.y = dy / time_diff_secs
                            carma_obj.velocity.twist.linear.z = dz / time_diff_secs
                        # If dt is too small, velocity remains zero or could use last known CARMA velocity
                        elif 'object_msg' in prev_data: # Check if 'object_msg' is in prev_data
                             carma_obj.velocity.twist.linear = prev_data['object_msg'].velocity.twist.linear
            
            carma_obj.confidence = 1.0 

            if len(carla_obj.shape.dimensions) >= 3:
                carma_obj.size.x = carla_obj.shape.dimensions[0]
                carma_obj.size.y = carla_obj.shape.dimensions[1]
                carma_obj.size.z = carla_obj.shape.dimensions[2]
            else:
                carma_obj.presence_vector &= ~ExternalObject.SIZE_PRESENCE_VECTOR
                self.get_logger().warn(f"Object ID {carma_obj.id} has insufficient shape dimensions. Size not set.")

            carma_obj.dynamic_obj = True

            if carla_obj.object_classified:
                classification_key = carla_obj.classification
                if classification_key in self.object_type_lookup:
                    carma_obj.object_type = self.object_type_lookup[classification_key]
                else:
                    carma_obj.object_type = ExternalObject.UNKNOWN
                    self.get_logger().debug(f"Obj ID {carma_obj.id} unmapped CARLA classification: {classification_key}, mapping to UNKNOWN.")
                carma_obj.presence_vector |= ExternalObject.OBJECT_TYPE_PRESENCE_VECTOR
            else:
                carma_obj.object_type = ExternalObject.UNKNOWN
                carma_obj.presence_vector &= ~ExternalObject.OBJECT_TYPE_PRESENCE_VECTOR
                self.get_logger().debug(f"Obj ID {carma_obj.id} not classified by input. Type not set.")

            return carma_obj
        except Exception as e:
            self.get_logger().error(f"Error converting object ID {carla_obj.id if carla_obj else 'UNKNOWN'}: {e}\n{traceback.format_exc()}")
            return None

    def cleanup_stale_objects(self):
        """Periodically called by a timer to remove objects that haven't been updated for several cycles."""
        with self.prev_objects_lock: 
            ids_to_remove = []
            for obj_id, data in self.prev_objects.items():
                if data.get('no_update_count', 0) > self.max_no_update_cycles:
                    ids_to_remove.append(obj_id)
            
            for obj_id in ids_to_remove:
                # Log before deleting, using the count from when it was marked for removal.
                # It's possible no_update_count changed again if callback ran between list creation and this loop,
                # but since the lock is held for this whole method now, this won't happen.
                count = self.prev_objects[obj_id].get('no_update_count', 'N/A')
                self.get_logger().info(f"Removing stale object ID {obj_id} due to inactivity (missed {count} update cycles).")
                if obj_id in self.prev_objects: # Should always be true here with the lock
                    del self.prev_objects[obj_id]

    def on_shutdown(self):
        """Called when the node is shutting down for cleanup."""
        self.get_logger().info(f"Shutting down {self.get_name()}.")
        if hasattr(self, 'cleanup_timer') and self.cleanup_timer and not self.cleanup_timer.is_canceled():
            self.cleanup_timer.cancel()

def main(args=None):
    """Main entry point for the ROS2 node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaToCarmaObjectsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info(f"{node.get_name()} shutting down due to KeyboardInterrupt.")
    except Exception as e:
        # Ensure logger is available or use print
        logger = rclpy.logging.get_logger("carla_to_carma_external_objects_node_main")
        if node : logger = node.get_logger()
        logger.error(f"Unhandled exception in {node.get_name() if node else 'CarlaToCarmaObjectsNode'}: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.on_shutdown() 
            node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()
