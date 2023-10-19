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

sudo add-apt-repository ppa:deadsnakes/ppa -y

sudo apt-get update && sudo apt-get install -y --no-install-recommends \
                            python3.7 \
                            python3.7-distutils \
                            python3-pip \
                            python3-wheel \
                            python3 \
                            python3-numpy \
                            libgps-dev \
                            ros-noetic-ackermann-msgs \
                            ros-noetic-derived-object-msgs \
                            ros-noetic-jsk-recognition-msgs \
                            ros-noetic-rqt \
                            ros-noetic-rviz \
                            wget


python3.7 -m pip install simple-pid
python3.7 -m pip install numpy --upgrade
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 10
alias python='/usr/bin/python3.7'
alias python3='/usr/bin/python3.7'


# Clone CARLA ROS bridge
sudo git clone --depth 1 -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# Clone ROS message
mkdir -p ~/msgs
if [ "${CARMA_VERSION}" = "develop" ]; then
  cd ~/msgs && sudo git clone --depth 1 --single-branch -b carma-develop https://github.com/usdot-fhwa-stol/autoware.ai.git
else
  cd ~/msgs && sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/autoware.ai.git
fi
cd ~/msgs && sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-msgs.git

# CARMA Utils package
mkdir -p ~/utils
cd ~/utils && sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git

# GPS Common
mkdir -p ~/gps && cd ~/gps 
sudo git clone https://github.com/swri-robotics/gps_umd.git

mkdir -p ~/carma_carla_ws/src/msgs && cd ~/carma_carla_ws/src/msgs

ln -s ~/msgs/carma-msgs/j3224_msgs
ln -s ~/msgs/carma-msgs/j2735_msgs
ln -s ~/msgs/carma-msgs/cav_msgs
ln -s ~/msgs/carma-msgs/can_msgs
ln -s ~/msgs/carma-msgs/cav_srvs
ln -s ~/msgs/carma-msgs/carma_cmake_common
ln -s ~/msgs/autoware.ai/messages/autoware_msgs

mkdir -p ~/carma_carla_ws/src/utils && cd ~/carma_carla_ws/src/utils 
ln -s ~/utils/carma-utils/carma_utils

cd ~/carma_carla_ws/src && 
ln -s ~/ros-bridge
ln -s ~/carma-carla-integration

cd ~/carma_carla_ws && /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
