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

echo "### Starting CARMA-CARLA Integration ROS 2 install ###"

# Add Python 3.10+ support if needed
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt-get update

# Install ROS 2 and CARLA dependencies
sudo apt-get install -y --no-install-recommends \
    libgps-dev \
    python3-distutils \
    python3-pip \
    ros-humble-ackermann-msgs \
    ros-humble-derived-object-msgs \
    ros-humble-rqt \
    ros-humble-rviz2 \
    wget \
    git

# Python dependencies
python3 -m pip install --upgrade pip
python3 -m pip install simple-pid==1.0.1 wheel numpy

# Ensure python points to python3
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Clone ROS 2 message packages
mkdir -p ~/msgs
if [ "${CARMA_VERSION}" = "develop-ros2" ]; then
  echo "Cloning carma-msgs develop branch (ROS 2 compatible)..."
  cd ~/msgs && git clone --depth 1 --branch develop https://github.com/usdot-fhwa-stol/carma-msgs.git
else
  echo "Cloning carma-msgs ${CARMA_VERSION} branch..."
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

# Link msgs and utils into carma-carla-integration src
mkdir -p ~/carma-carla-integration/src
ln -sf ~/msgs/carma-msgs ~/carma-carla-integration/src/
ln -sf ~/utils/carma-utils ~/carma-carla-integration/src/

# Remove ROS 1-only packages if they exist
echo "Removing ROS 1-only packages..."
rm -rf ~/carma-carla-integration/src/cav_msgs || true
rm -rf ~/carma-carla-integration/src/cav_srvs || true
rm -rf ~/carma-carla-integration/src/carma_debug_msgs || true
rm -rf ~/carma-carla-integration/src/carla-sensor-lib/carla_sensors_integration || true

# Build using colcon
cd ~/carma-carla-integration
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "Starting colcon build..."
colcon build --symlink-install --packages-skip cav_msgs cav_srvs carma_debug_msgs carla_sensors_integration || true

echo "Sourcing workspace..."
source install/setup.bash

echo "### CARMA-CARLA Integration ROS 2 install completed successfully! ###"
