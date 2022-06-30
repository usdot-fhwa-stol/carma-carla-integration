#!/bin/bash

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
       $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING
