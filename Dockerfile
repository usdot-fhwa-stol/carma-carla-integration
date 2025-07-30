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

# ================================
# Base Image
# ================================
ARG DOCKER_ORG="usdotfhwastoldev"
ARG DOCKER_TAG="develop-humble"
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG}
ARG GIT_BRANCH="develop-ros2"
ENV CARMA_VERSION=${GIT_BRANCH}

LABEL Description="Dockerized CARMA-CARLA integration (ROS 2 + CARLA ROS 2 Bridge UE5 0.10.0)"

# ================================
# Metadata Labels
# ================================
ARG VERSION
ARG VCS_REF
ARG BUILD_DATE
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-carla-integration"
LABEL org.label-schema.description="ROS 2 CARMA-CARLA integration image with CARLA ROS 2 Bridge support (UE5 CARLA 0.10.0)"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-simulation/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# ================================
# User Setup & System Dependencies
# ================================
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-distutils \
    libgps-dev \
    libvulkan1 \
    libsdl2-2.0-0 \
    ros-humble-ackermann-msgs \
    ros-humble-derived-object-msgs \
    ros-humble-rqt \
    ros-humble-rviz2 \
    wget git && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install simple-pid==1.0.1 wheel numpy

# ================================
# Copy CARLA PythonAPI (Local)
# ================================
USER carma
WORKDIR /home/carma

# Copy the PythonAPI folder from repo
COPY --chown=carma:carma PythonAPI ./PythonAPI

# Install CARLA Python API from its .whl
RUN CARLA_WHL=$(find PythonAPI/carla/dist -name "*.whl") && \
    if [ -z "$CARLA_WHL" ]; then \
        echo "ERROR: No CARLA .whl file found in PythonAPI/carla/dist"; \
        exit 1; \
    fi && \
    python3 -m pip install $CARLA_WHL

# Set CARLA Python API environment variables
ENV CARLA_VERSION=0.10.0
ENV CARLA_PYTHONAPI=/home/carma/PythonAPI
ENV PYTHONPATH=$CARLA_PYTHONAPI/carla/dist:$CARLA_PYTHONAPI:$PYTHONPATH

# ================================
# Workspace Setup
# ================================
COPY --chown=carma:carma docker ./docker
COPY --chown=carma:carma carma-carla-integration ./carma-carla-integration

# Install ROS 2 dependencies and build
RUN /home/carma/docker/install.sh

# ================================
# Final Settings
# ================================
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
CMD ["bash"]