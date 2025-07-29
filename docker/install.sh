#!/bin/bash

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
  cd ~/msgs && git clone --depth 1 --branch develop https://github.com/usdot-fhwa-stol/carma-msgs.git
else
  cd ~/msgs && git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-msgs.git
fi

# Clone CARMA utils (ROS 2)
mkdir -p ~/utils && cd ~/utils
git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git

# Clone CARLA Sensor Lib
cd ~
git clone --depth 1 --branch ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carla-sensor-lib.git

# Link msgs and utils into carma-carla-integration src
ln -s ~/msgs/carma-msgs ~/carma-carla-integration/src/
ln -s ~/utils/carma-utils ~/carma-carla-integration/src/

# Build using colcon
cd ~/carma-carla-integration
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash