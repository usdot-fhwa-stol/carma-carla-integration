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


#PYTHON 3.7 
RUN sudo add-apt-repository ppa:deadsnakes/ppa -y
RUN sudo apt update
RUN sudo apt install python3.7 -y
RUN sudo apt-get install python3.7-distutils -y
RUN sudo python3.7 -m pip install numpy --upgrade
RUN alias python='/usr/bin/python3.7'

# CARLA PythonAPI
RUN sudo mkdir ./PythonAPI
COPY PythonAPI ./PythonAPI
ADD https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg ./PythonAPI


#RUN sudo apt-get update && apt-get install -y \
#		git \
#		curl \
#		wget \
#		nano \
#		libpng16-16 \
#		libsdl2-2.0 \
#		libgeographic-dev \
#		libpugixml-dev \
#		software-properties-common

# Update gcc and g++
#RUN sudo add-apt-repository ppa:ubuntu-toolchain-r/test
#RUN sudo apt-get update && apt-get install -y \
#		gcc-6 \
#		g++-6

# CARLA ROS Bridge
RUN sudo git clone --depth 1 -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARMA-CARLA integration tool copy from local
COPY carma-carla-integration ./carma-carla-integration

#COPY carma_utils ./../carma-utils/carma_utils

# CARMA-CARLA integration tool necessary package and msgs
#RUN apt-get install -y --no-install-recommends \
 #   python-pip \
  #  python-wheel \
   # ros-kinetic-ackermann-msgs \
#    ros-kinetic-derived-object-msgs \
 #   ros-kinetic-jsk-recognition-msgs \
#		ros-kinetic-rqt \
#		ros-kinetic-rviz \
#		ros-kinetic-pcl-conversions \
#		ros-kinetic-pcl-ros \
#		ros-kinetic-cv-bridge

RUN sudo apt-get install -y --no-install-recommends \
    python3-pip \
#    python-wheel \
	python \
	python3 \
	python3-numpy \
	libgps-dev \
    ros-noetic-ackermann-msgs \
    ros-noetic-derived-object-msgs \
    ros-noetic-jsk-recognition-msgs \
	ros-noetic-rqt \
	ros-noetic-rviz 

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
#CARMA Utils package
RUN sudo mkdir -p utils
RUN  cd utils \
		&& sudo git clone --depth 1 --single-branch -b ${CARMA_VERSION} https://github.com/usdot-fhwa-stol/carma-utils.git

#GPS Common
RUN sudo mkdir -p gps
RUN cd gps \
	&& sudo git clone https://github.com/swri-robotics/gps_umd.git
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


RUN cd carma_carla_ws/src \
    && sudo ln -s ../../ros-bridge \
    && sudo ln -s ../../carma-carla-integration \
    && cd .. \
    && sudo /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

RUN sudo pip install simple-pid

CMD ["/bin/bash"]
