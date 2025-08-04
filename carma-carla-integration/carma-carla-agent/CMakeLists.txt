# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
cmake_minimum_required(VERSION 3.8)
project(carma_carla_agent)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY agent/
  DESTINATION share/${PROJECT_NAME}/agent
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export package information
ament_package()