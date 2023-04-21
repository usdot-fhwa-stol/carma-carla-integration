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
WORKDIR /home

ARG CARMA_VERSION="carma-system-4.2.0"

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
	ros-noetic-rviz \
	wget

RUN sudo python3.7 -m pip install simple-pid
RUN sudo python3.7 -m pip install numpy --upgrade
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
RUN alias python='/usr/bin/python3.7'

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


RUN sudo mkdir -p carma_carla_ws/src/msgs

RUN  cd carma_carla_ws/src/msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/j2735_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/cav_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/can_msgs \
		&& sudo ln -s ../../../msgs/carma-msgs/cav_srvs \
    && sudo ln -s ../../../msgs/carma-msgs/carma_cmake_common \
		&& sudo ln -s ../../../msgs/autoware.ai/messages/autoware_msgs


RUN sudo mkdir -p carma_carla_ws/src/utils \
		&& cd carma_carla_ws/src/utils \
		&& sudo ln -s /home/utils/carma-utils/carma_utils


RUN cd carma_carla_ws/src \
    && sudo ln -s ../../ros-bridge \
    && sudo ln -s ../../carma-carla-integration \
    && cd .. \
    && sudo /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

CMD ["/bin/bash"]
