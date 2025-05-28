#!/usr/bin/env python3
# Copyright 2025 Will Varner, UGA MSC Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Utility functions for CARLA-ROS 2 bridge interactions, primarily for establishing
and managing connections to the CARLA simulator.
"""

import carla
import traceback # For detailed exception logging

def connect_to_carla_server(node_logger, host: str, port: int, timeout: float) -> tuple[carla.Client | None, carla.World | None]:
    """
    Establishes a passive connection to the CARLA server.

    This function attempts to connect to the CARLA server at the specified
    host and port. It does not attempt to change world settings (e.g.,
    synchronous mode or fixed_delta_seconds), assuming CARLA (e.g., when
    launched with --ros2) or another dedicated node is managing the
    simulation tick and world settings.

    Args:
        node_logger: The logger object from the calling ROS 2 node.
        host: The hostname or IP address of the CARLA server.
        port: The TCP port of the CARLA server.
        timeout: The connection timeout in seconds.

    Returns:
        A tuple containing (carla.Client, carla.World) if successful,
        otherwise (None, None).
    """
    #I am 99% sure that using the --ros2 string on ./CarlaUnreal.sh controls the tick, so we dont need to set anything on each node init
    node_logger.info(f"Attempting passive connection to CARLA server at {host}:{port}...")
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        world = client.get_world()
        # A simple call to verify the connection is alive and world is accessible
        world.get_snapshot()
        node_logger.info(f"Passively connected to CARLA server (Host: {host}, Port: {port}).")
        return client, world
    except RuntimeError as e:
        node_logger.error(f"Failed to connect passively to CARLA: {e}. Is the server running correctly at {host}:{port}?")
        return None, None
    except Exception as e:
        node_logger.error(f"An unexpected error occurred during passive CARLA connection: {e}\n{traceback.format_exc()}")
        return None, None
    #Feel free to add any other shared or common functions, right now this solely houses the connect script but can easily be exapanded