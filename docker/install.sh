#  Copyright (C) 2018-2020 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

#!/bin/bash
set -e

echo "### Starting CARMA-CARLA Integration ROS 2 Workspace Setup ###"

echo "Cleaning workspace..."
rm -rf ~/carma-carla-integration/build ~/carma-carla-integration/install ~/carma-carla-integration/log

# Install System Dependencies
echo "Installing system dependencies..."
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    python3-pip \
    python3-distutils \
    libgps-dev \
    libvulkan1 \
    libsdl2-2.0-0 \
    ros-humble-ackermann-msgs \
    ros-humble-derived-object-msgs \
    ros-humble-rqt \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    wget git && \
    sudo rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python dependencies
# python3 -m pip install --upgrade pip
# python3 -m pip install simple-pid==1.0.1 wheel numpy

# Ensure python points to python3
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Install CARLA PythonAPI
echo "Installing CARLA Python API (UE5 0.10.0)..."
CARLA_WHL=$(find ~/PythonAPI/carla/dist -name "*.whl")
if [ -z "$CARLA_WHL" ]; then
    echo "ERROR: CARLA .whl file not found in ~/PythonAPI/carla/dist"
    exit 1
fi
python3 -m pip install "$CARLA_WHL"

# Source ROS 2 Environment
echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash

# Clone ROS 2 message packages
mkdir -p ~/msgs
if [ "${CARMA_VERSION}" = "develop-ros2" ]; then
  echo "Cloning carma-msgs (develop branch, ROS 2 compatible)..."
  cd ~/msgs && git clone --depth 1 --branch develop https://github.com/usdot-fhwa-stol/carma-msgs.git
else
  echo "Cloning carma-msgs (${CARMA_VERSION} branch)..."
  cd ~/msgs && git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-msgs.git
fi

# Clone CARMA utils (ROS 2)
mkdir -p ~/utils && cd ~/utils
if [ "${CARMA_VERSION}" = "develop-ros2" ]; then
  git clone --depth 1 --branch develop https://github.com/usdot-fhwa-stol/carma-utils.git
else
  git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git
fi

# Clone CARLA Sensor Lib
cd ~
if [ "${CARMA_VERSION}" = "develop-ros2" ]; then
  git clone --depth 1 --branch develop https://github.com/usdot-fhwa-stol/carla-sensor-lib.git
else
  git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carla-sensor-lib.git
fi

# Clone autoware_msgs package (includes jsk_recognition_msgs)
echo "Cloning autoware_msgs from CARMA develop branch..."
mkdir -p ~/autoware_msgs && cd ~/autoware_msgs
git clone --depth 1 --branch carma-develop https://github.com/usdot-fhwa-stol/autoware.ai.git

# Clone carla_ackermann_msgs
echo "Cloning carla_ackermann_msgs..."
mkdir -p ~/carla_ackermann_msgs && cd ~/carla_ackermann_msgs
git clone --depth 1 --branch master https://github.com/carla-simulator/ros-bridge.git

# Prepare workspace
echo "Setting up carma-carla-integration workspace..."
mkdir -p ~/carma-carla-integration/src

# Clean up old symlinks if they exist
rm -rf ~/carma-carla-integration/src/carma-msgs || true
rm -rf ~/carma-carla-integration/src/carma-utils || true

# Symlink message and utility packages into workspace
ln -sf ~/msgs/carma-msgs ~/carma-carla-integration/src/
ln -sf ~/utils/carma-utils ~/carma-carla-integration/src/
ln -sf ~/autoware_msgs/autoware.ai/messages/autoware_msgs ~/carma-carla-integration/src/
ln -sf ~/autoware_msgs/autoware.ai/jsk_recognition/jsk_recognition_msgs ~/carma-carla-integration/src/
ln -sf ~/carla_ackermann_msgs/ros-bridge/carla_ackermann_msgs ~/carma-carla-integration/src/

# Remove ROS 1-only packages
echo "Removing ROS 1-only packages (if present)..."
rm -rf ~/carma-carla-integration/src/cav_msgs || true
rm -rf ~/carma-carla-integration/src/cav_srvs || true
rm -rf ~/carma-carla-integration/src/carma_debug_msgs || true
rm -rf ~/carla-sensor-lib/carla_sensors_integration || true

# Colcon Build (Dependency Order)
cd ~/carma-carla-integration

echo "Building core message packages..."
colcon list
colcon build --symlink-install --packages-select \
    carma_cmake_common \
    carma_msgs \
    carma_perception_msgs \
    j2735_v2x_msgs \
    j3224_v2x_msgs \
    jsk_recognition_msgs \
    carma_driver_msgs \
    carma_v2x_msgs \
    carma_planning_msgs \
    carla_msgs \
    autoware_msgs \
    carla_ackermann_msgs

echo "Building CARLA ROS 2 Bridge..."
colcon build --symlink-install --packages-select carla_ros2_bridge

echo "Building remaining dependencies..."
colcon build --symlink-install --packages-up-to carma_carla_bridge

echo "Finalizing build..."
colcon build --symlink-install --packages-select carma_carla_bridge

echo "Installing Python dependencies from requirements.txt..."
python3 -m pip install -r ~/carma-carla-integration/carla-ros2-bridge/requirements.txt

echo "### CARMA-CARLA Integration ROS 2 Workspace Setup Completed Successfully! ###"
