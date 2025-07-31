# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

cmake_minimum_required(VERSION 3.5)
project(carma_carla_bridge)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(carma_driver_msgs REQUIRED)
find_package(carma_perception_msgs REQUIRED)
find_package(carma_planning_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/carma_carla_bridge/carma_carla_driver_status.py
  src/carma_carla_bridge/carma_carla_robot_status.py
  src/carma_carla_bridge/carla_to_carma_vehicle_status.py
  src/carma_carla_bridge/carla_to_carma_localization.py
  src/carma_carla_bridge/carma_to_carla_ackermann_cmd.py
  src/carma_carla_bridge/carla_to_carma_external_objects.py
  src/carma_carla_bridge/carma_carla_route.py
  src/carma_carla_bridge/carma_carla_plugins.py
  src/carma_carla_bridge/carma_carla_guidance.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()