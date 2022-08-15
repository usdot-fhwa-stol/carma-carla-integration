#!/bin/bash

carma__start_sim_mode(){
  bash ./docker_compose_generator.sh
  echo "Starting CARMA Platform foreground processes..."
  docker run -v /opt/carma/simulation/docker-compose.yml:/opt/carma/vehicle/config/docker-compose.yml --rm --volumes-from carma-config:ro --entrypoint sh busybox:latest -c \
  'cat /opt/carma/vehicle/config/docker-compose.yml' | \
  docker-compose -f - -p carma up $@
}


carma__start_sim_mode
