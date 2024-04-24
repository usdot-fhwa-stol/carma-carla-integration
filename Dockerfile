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
ARG DOCKER_ORG="usdotfhwastoldev"
ARG DOCKER_TAG="develop"
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG} 
ARG GIT_BRANCH="develop" 
ENV CARMA_VERSION=${GIT_BRANCH}

LABEL Description="Dockerised CARMA-CARLA integration"

ARG VERSION
ARG VCS_REF
ARG BUILD_DATE
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-carla-integration"
LABEL org.label-schema.description="XIL carma-carla-integration for evaluation and testing of the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-simulation/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

USER root
RUN apt-get update && apt-get install -y sudo

USER carma
WORKDIR /home/carma
COPY --chown=carma:carma /docker ./docker
COPY PythonAPI ./PythonAPI
COPY /carma-carla-integration ./carma-carla-integration

# Run the install script as root
#TODO (SIM-5) Fix install/checkout scripts
RUN /home/carma/docker/install.sh
COPY /patch/settings.yaml ./ros-bridge/carla_ros_bridge/config
RUN rm -R -rf /home/carma/docker

ENV LOAD_CARLA_EGG=True
ENV CARLA_VERSION=0.9.10
ENV CARLA_EGG_DIR=/home/carma/PythonAPI/carla/dist

CMD ["/bin/bash"]

