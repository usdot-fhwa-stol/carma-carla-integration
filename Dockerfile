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
FROM usdotfhwastol/carma-base:carma-system-4.2.0
WORKDIR /home

ARG CARMA_VERSION="carma-system-3.9.0"
ARG CARMA_CONFIG_BRANCH="feature/add_datastream_toggle"

# CARLA PythonAPI
COPY PythonAPI ./PythonAPI

RUN sudo add-apt-repository ppa:deadsnakes/ppa -y

# CARLA ROS Bridge
RUN sudo git clone --depth 1 -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARMA-CARLA integration tool copy from local
COPY carma-carla-integration ./carma-carla-integration

RUN sudo apt-get install -y --no-install-recommends \
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
  	ros-noetic-rviz

RUN sudo python3.7 -m pip install simple-pid
RUN sudo python3.7 -m pip install numpy --upgrade
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
RUN alias python='/usr/bin/python3.7'


# Upgrade CMake to 3.13
RUN sudo wget https://cmake.org/files/v3.13/cmake-3.13.0-Linux-x86_64.tar.gz
RUN sudo tar -xzvf cmake-3.13.0-Linux-x86_64.tar.gz
RUN sudo mv cmake-3.13.0-Linux-x86_64 /opt/cmake-3.13.0
RUN sudo ln -sf /opt/cmake-3.13.0/bin/* /usr/bin/
RUN sudo rm cmake-3.13.0-Linux-x86_64.tar.gz

# Clone ROS message
RUN sudo mkdir -p msgs
RUN cd msgs \
		&& sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/autoware.ai.git \
		&& sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-msgs.git

#CARMA Utils package
RUN sudo mkdir -p utils
RUN  cd utils \
		&& sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git

#GPS Common
RUN sudo mkdir -p gps
RUN cd gps \
	&& sudo git clone https://github.com/swri-robotics/gps_umd.git

#Carma-Config
RUN sudo mkdir -p config
RUN cd config \
	&& sudo git clone --depth 1 --single-branch -b ${CARMA_CONFIG_BRANCH} https://github.com/usdot-fhwa-stol/carma-config.git

# Catkin make for both ros-bridge and carma-carla-integration
RUN sudo mkdir -p carma_carla_ws/src/msgs

RUN  cd carma_carla_ws/src/msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/j2735_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/cav_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/can_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/cav_srvs \
		&& sudo ln -s ../../../msgs/autoware.ai/messages/autoware_msgs

RUN sudo mkdir -p carma_carla_ws/src/utils \
		&& cd carma_carla_ws/src/utils \
		&& sudo ln -s /home/utils/carma-utils/carma_utils

#Config
RUN sudo mkdir -p carma_carla_ws/src/config \
		&& cd carma_carla_ws/src/config \
		&& sudo ln -s /home/config/carma-config/carla_integration

RUN cd carma_carla_ws/src \
    && sudo ln -s ../../ros-bridge \
    && sudo ln -s ../../carma-carla-integration \
    && cd .. \
    && sudo /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

CMD ["/bin/bash"]
