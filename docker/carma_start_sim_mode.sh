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

carma__start_sim_mode(){
  # check if folder exists
  if [ ! -d '/opt/carma-simulation' ]; then
    echo "Simulation not found in carma folder, creating carma-simulation folder"
    sudo mkdir -m 777 -p /opt/carma-simulation
    echo "Done"
  fi

  if [ ! -f '/opt/carma-simulation/global_config.json' ]; then
    echo "Copying config to /opt/carma-simulation..."
    cp ../config/global_config.json /opt/carma-simulation
    echo "Done"
  fi
  if [ ! -f '/opt/carma-simulation/vehicle_config.json' ]; then
    echo "Copying config to /opt/carma-simulation..."
    cp ../config/vehicle_config.json /opt/carma-simulation
    echo "Done"
  fi

  echo "Generating docker-compose.yml to /opt/carma/simulation..."
  ./docker_compose_generator.sh > /opt/carma-simulation/docker-compose.yml
  echo "Done"

  echo "Starting CARMA Platform foreground processes..."
  cat /opt/carma-simulation/docker-compose.yml | docker-compose -f - -p carma up $@
}

carma__start_sim_mode
