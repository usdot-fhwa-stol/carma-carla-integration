# Copyright (C) 2021 LEIDOS.
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
FROM usdotfhwastoldev/carma-base:develop

# CARLA PythonAPI
COPY PythonAPI ./PythonAPI

# CARMA-CARLA integration tool copy from local
COPY carma-carla-integration ./carma-carla-integration

# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

ARG CARMA_VERSION="carma-system-4.3.0"

ARG DEPENDENCIES="python3.7 \
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
    ros-noetic-rviz"

RUN sudo add-apt-repository -u -y ppa:deadsnakes/ppa && \
    sudo apt-get install -y --no-install-recommends ${DEPENDENCIES} && \
    # Install Python dependencies
    sudo python3.7 -m pip install simple-pid && \
    sudo python3.7 -m pip install numpy --upgrade && \
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10 && \
    alias python='/usr/bin/python3.7' && \
    # CARLA ROS Bridge
    sudo git clone --depth 1 -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git /home/ros-bridge && \
    # Clone ROS message
    sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/autoware.ai.git /home/msgs/autoware.ai && \
    sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-msgs.git /home/msgs/carma-msgs && \
    #CARMA Utils package
    sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git /home/utils/carma-utils && \
    #GPS Common
    sudo mkdir -p /home/gps && \
    cd /home/gps && \
    sudo git clone https://github.com/swri-robotics/gps_umd.git && \
    # msgs
    sudo mkdir -p /home/carma_carla_ws/src/msgs && \
    sudo ln -s /home/carma_carla_ws/src/msgs/carma-msgs/j2735_msgs /home/carma_carla_ws/src/j2735_msgs && \
    sudo ln -s /home/carma_carla_ws/src/msgs/carma-msgs/cav_msgs /home/carma_carla_ws/src/cav_msgs && \
    sudo ln -s /home/carma_carla_ws/src/msgs/carma-msgs/can_msgs /home/carma_carla_ws/src/can_msgs && \
    sudo ln -s /home/carma_carla_ws/src/msgs/carma-msgs/cav_srvs /home/carma_carla_ws/src/cav_srvs && \
    sudo ln -s /home/carma_carla_ws/src/msgs/carma-msgs/carma_cmake_common /home/carma_carla_ws/src/carma_cmake_common && \
    sudo ln -s /home/carma_carla_ws/src/msgs/autoware.ai/messages/autoware_msgs /home/carma_carla_ws/src/autoware_msgs && \
    # utils
    sudo mkdir -p /home/carma_carla_ws/src/utils && \
    sudo ln -s /home/utils/carma-utils/carma_utils && \
    sudo ln -s /home/carma_carla_ws/ros-bridge /home/carma_carla_ws/src/ros-bridge && \
    sudo ln -s /home/carma_carla_ws/carma-carla-integration /home/carma_carla_ws/src/carma-carla-integration && \
    cd /home/carma_carla_ws/ && \
    sudo /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

CMD ["/bin/bash"]
