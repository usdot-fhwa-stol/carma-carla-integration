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
  bash ./docker_compose_generator.sh
  echo "Starting CARMA Platform foreground processes..."
  docker run -v /opt/carma/simulation/docker-compose.yml:/opt/carma/vehicle/config/docker-compose.yml --rm --volumes-from carma-config:ro --entrypoint sh busybox:latest -c \
  'cat /opt/carma/vehicle/config/docker-compose.yml' | \
  docker-compose -f - -p carma up $@
}


carma__start_sim_mode
