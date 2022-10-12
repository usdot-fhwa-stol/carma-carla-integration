#!/bin/bash
#  Copyright (C) 2018-2021 LEIDOS.
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

RUNTIME=""
IMAGE=$(./get-image-name.sh | tr '[:upper:]' '[:lower:]')
USERNAME=usdotfhwastol
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
            ;;
    esac
done
echo "##### Running usdotfhwastol/$IMAGE:$COMPONENT_VERSION_STRING docker container #####"

docker run \
       -it --rm\
       --name carma-carla-integration \
       --net=host \
       --volumes-from=carma-config \
       $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING
