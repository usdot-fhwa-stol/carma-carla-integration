# CARLA ROS 2 Bridge (for CARLA 0.10.0+)

This package provides a ROS 2 bridge for the CARLA simulator (specifically targeting CARLA 0.10.0 and newer running on UE5), allowing for the publishing of key vehicle and simulation data to ROS 2 topics. It is developed by Will Varner at the UGA MSC Lab.

This bridge is intended to provide topics similar to those from older CARLA ROS 1 bridges, facilitating the migration of AD stacks like [CDASim](https://github.com/usdot-fhwa-stol/cdasim) to ROS 2 with newer CARLA versions.

## Features

Currently, the bridge provides the following publishers:

* **Vehicle Information:** Publishes static physical and descriptive information about the ego vehicle.
    * Topic: `/carla/{role_name}/vehicle_info`
    * Message Type: `carla_msgs/msg/CarlaEgoVehicleInfo`
    * Node: `vehicle_info_publisher_node`
* **Vehicle Status:** Publishes dynamic status of the ego vehicle (speed, acceleration, orientation, control inputs).
    * Topic: `/carla/{role_name}/vehicle_status`
    * Message Type: `carla_msgs/msg/CarlaEgoVehicleStatus`
    * Node: `vehicle_status_publisher_node`
* **Odometry:** Publishes the ego vehicle's odometry (pose and twist in the world frame).
    * Topic: `/carla/{role_name}/odometry`
    * Message Type: `nav_msgs/msg/Odometry`
    * Node: `odometry_publisher_node`
* **Objects:** Publishes an array of detected dynamic objects (other vehicles, pedestrians) in the world.
    * Topic: `/carla/{role_name}/objects`
    * Message Type: `derived_object_msgs/msg/ObjectArray`
    * Node: `objects_publisher_node`

## Prerequisites

* ROS 2 Humble
* CARLA Simulator (0.10.0+)
* CARLA Python API for the corresponding CARLA version
* `carla_msgs` (compatible with your CARLA version and ROS 2 Humble)
* `derived_object_msgs` (for ROS 2 Humble)
* Python dependencies: `numpy`, `transforms3d` (installable via pip)

## Build Instructions

1.  **Clone Required Message Packages (if not already present):**
    Make sure you have `ros-carla-msgs` and `astuff_sensor_msgs` (which contains `derived_object_msgs`) in your ROS 2 workspace's `src` directory.
    ```bash
    cd your_ros2_ws/src
    git clone [https://github.com/carla-simulator/ros-carla-msgs.git](https://github.com/carla-simulator/ros-carla-msgs.git)
    git clone [https://github.com/astuff/astuff_sensor_msgs.git](https://github.com/astuff/astuff_sensor_msgs.git)
    # Ensure you check out branches/tags compatible with ROS 2 Humble for these packages.
    ```

2.  **Clone this Bridge Package:**
    Clone this `carla_ros2_bridge` repository into your ROS 2 workspace's `src` directory.
    ```bash
    cd your_ros2_ws/src
    # git clone <URL_to_this_repository>
    ```

3.  **Install Python Dependencies:**
    ```bash
    pip install transforms3d numpy
    ```

4.  **Build the Workspace:**
    Navigate to the root of your ROS 2 workspace and build the packages:
    ```bash
    cd your_ros2_ws
    colcon build --symlink-install
    ```
    If you only want to build specific packages:
    ```bash
    colcon build --packages-select carla_msgs derived_object_msgs carla_ros2_bridge --symlink-install
    ```

5.  **Source the Workspace:**
    ```bash
    source your_ros2_ws/install/setup.bash
    ```

## Running the Bridge

1.  **Start CARLA Server:**
    Launch your CARLA 0.10.0+ simulator. using the `--ros2` flag lets us use CARLA 10's new native ROS 2 capabilities for raw sensor data alongside this bridge.
    ```bash
    ./CarlaUE5.sh --ros2
    ```

2.  **Spawn Ego Vehicle:**
    Ensure a vehicle with the configured `ego_vehicle_role_name` (default: "hero") is spawned in the CARLA simulation. This can be done using a CARLA client script (e.g., CARLA's built-in `PythonAPI/examples/ros2/ros2_native.py` with a suitable JSON configuration).

3.  **Launch the Bridge Nodes:**
    A basic launch file is provided to start all publisher nodes:
    ```bash
    ros2 launch carla_ros2_bridge carla_bridge_launch.py ego_vehicle_role_name:=hero
    ```
    You can also run nodes individually if needed:
    ```bash
    ros2 run carla_ros2_bridge odometry_publisher_node --ros-args -p ego_vehicle_role_name:=hero
    ```

## Parameters

Each publisher node accepts the following common ROS 2 parameters:

* `carla_host` (string, default: "localhost"): Hostname of the CARLA server.
* `carla_port` (int, default: 2000): TCP port of the CARLA server.
* `ego_vehicle_role_name` (string, default: "hero"): The `role_name` attribute of the ego vehicle in CARLA. Used for namespacing topics and identifying the primary vehicle.
* `update_frequency` (double, default varies per node): Publishing frequency in Hz. For `vehicle_info_publisher_node`, a value <= 0.0 means publish once.
* `carla_timeout` (double, default: 10.0): Timeout for connecting to the CARLA server in seconds.

Specific nodes may have additional parameters (e.g., `world_frame_id` for odometry and objects).

## Future Work & To-Do

- [ ] Addition of TF (Transform) publishers.
- [ ] Support for dynamic spawning/destruction of ego vehicles.

## License

Created by Will Varner in UGA's MSC Lab under Dr. Yunli Shao
This project is licensed under the Apache License 2.0. Check the LICENSE file for details.
