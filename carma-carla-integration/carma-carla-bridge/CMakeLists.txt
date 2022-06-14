# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

cmake_minimum_required(VERSION 2.8.3)
project(carma_carla_bridge)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  rospy
  cav_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    src/carma_carla_bridge/carma_carla_driver_status
    src/carma_carla_bridge/carma_carla_robot_status
    src/carma_carla_bridge/carla_to_carma_vehicle_status
    src/carma_carla_bridge/carla_to_carma_localization
    src/carma_carla_bridge/carma_to_carla_ackermann_cmd
    src/carma_carla_bridge/carla_to_carma_external_objects
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
