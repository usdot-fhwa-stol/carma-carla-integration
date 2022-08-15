#!/bin/bash

docker_compose_generater() {
  local CARLA_SIM_VERSION=""
  local CARMA_CARLA_VERSION=""
  local CARMA_VERSION=""
  local USERNAME=""
  local TOWN=""
  local INSTANCES=0

  while read -r line; do
    if [[ $line =~ .'carla_sim_version'. ]]; then
      [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && CARLA_SIM_VERSION=( "${BASH_REMATCH[1]}" )
    elif [[ $line =~ .'carma_carla_version'. ]]; then
      [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && CARMA_CARLA_VERSION=( "${BASH_REMATCH[1]}" )
    elif [[ $line =~ .'carma_version'. ]]; then
      [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && CARMA_VERSION=( "${BASH_REMATCH[1]}" )
    elif [[ $line =~ .'username'. ]]; then
      [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && USERNAME=( "${BASH_REMATCH[1]}" )
    elif [[ $line =~ .'town'. ]]; then
      [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && TOWN=( "${BASH_REMATCH[1]}" )
    fi
  done < ../config/global_config.json

  declare -a VEHICLE_CONFIG_ARR
  while read -r line; do
    [[ $line =~ :[[:blank:]]+\"(.*)\" ]] && VEHICLE_CONFIG_ARR+=( "${BASH_REMATCH[1]}" )
    if [[ $line =~ .'role_name'. ]]; then
      let "INSTANCES+=1"
    fi
  done < ../config/vehicle_config.json

  if [ $INSTANCES == 0 ]; then
    echo "No vehicle config detected, check the vehicle config file at /carma-carla-integration/config/"
    exit -1
  fi

  cat <<HEADER
version: '3'
services:
HEADER

cat <<BLOCK
  carlasim:
    privileged: true
    image: carlasim/$CARLA_SIM_VERSION
    networks:
      carma_sim_net:
        ipv4_address: 172.2.0.2
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
    command: bash -c "./CarlaUE4.sh -opengl -ResX=1080 -ResY=720"

  carla_map_loading:
    image: $USERNAME/carma-carla-integration:$CARMA_CARLA_VERSION
    networks:
      carma_sim_net:
        ipv4_address: 172.2.0.3
    command: bash -c "cd /home/PythonAPI/util && python map_loading.py --host 172.2.0.2 -m $TOWN"

BLOCK

  for i in $(seq $INSTANCES); do
    cat <<BLOCK
  roscore_$i:
    image: $USERNAME/carma-base:$CARMA_VERSION
    depends_on:
      carla_map_loading:
        condition: service_completed_successfully
    networks:
      carma_net_$i:
        ipv4_address: 172.$(($i+2)).0.2
    volumes_from:
      - container:carma-config:ro
    volumes:
      - /opt/carma/.ros:/home/carma/.ros
      - /opt/carma/logs:/opt/carma/logs
    restart: always
    environment:
      - ROS_IP=172.$(($i+2)).0.2
      - ROS_MASTER_URI=http://172.$(($i+2)).0.2:11311/
    command: roscore

  platform_$i:
    image: $USERNAME/carma-platform:$CARMA_VERSION
    networks:
      carma_net_$i:
        ipv4_address: 172.$(($i+2)).0.3
    container_name: platform_$i
    volumes_from:
      - container:carma-config:ro
    volumes:
      - /opt/carma/logs:/opt/carma/logs
      - /opt/carma/.ros:/home/carma/.ros
      - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
      - /opt/carma/yolo:/opt/carma/yolo
      - /opt/carma/maps:/opt/carma/maps
      - /opt/carma/routes:/opt/carma/routes
      - /opt/carma/data:/opt/carma/data
      - /opt/carma/simulation:/opt/carma/simulation
    environment:
      - ROS_IP=172.$(($i+2)).0.3
      - ROS_MASTER_URI=http://172.$(($i+2)).0.2:11311/
    command: bash -c 'wait-for-it.sh 172.$(($i+2)).0.2:11311 -- roslaunch /opt/carma/vehicle/config/carma_docker.launch'

  carma-carla-integration_$i:
    image: $USERNAME/carma-carla-integration:$CARMA_CARLA_VERSION
    depends_on:
      carla_map_loading:
        condition: service_completed_successfully
    networks:
      carma_net_$i:
        ipv4_address: 172.$(($i+2)).0.4
      carma_sim_net:
        ipv4_address: 172.2.0.$(($i+3))
    environment:
      - ROS_IP=172.$(($i+2)).0.4
      - ROS_MASTER_URI=http://172.$(($i+2)).0.2:11311/
    command: bash -c "export PYTHONPATH=$PYTHONPATH:/home/PythonAPI/carla/dist/carla-0.9.10-py2.7-linux-x86_64.egg &&
                      source /home/carma_carla_ws/devel/setup.bash &&
                      ./wait-for-it.sh 172.$(($i+2)).0.3:11311 --
                      roslaunch carma_carla_agent carma_carla_agent.launch role_name:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15)) ]}'
                                                                           town:='$TOWN'
                                                                           host:='172.2.0.2'
                                                                           spawn_point:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 1)) ]}'
                                                                           selected_route:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 2)) ]}'
                                                                           selected_plugins:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 3)) ]}'
                                                                           speed_Kp:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 4)) ]}'
                                                                           speed_Ki:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 5)) ]}'
                                                                           speed_Kd:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 6)) ]}'
                                                                           accel_Kp:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 7)) ]}'
                                                                           accel_Ki:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 8)) ]}'
                                                                           accel_Kd:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 9)) ]}'
                                                                           init_speed:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 10)) ]}'
                                                                           init_acceleration:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 11)) ]}'
                                                                           init_steering_angle:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 12)) ]}'
                                                                           init_jerk:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 13)) ]}'
                                                                           max_steering_degree:='${VEHICLE_CONFIG_ARR[ $(( $(($i - 1)) * 15 + 14)) ]}'"

BLOCK
  done
cat <<HEADER
networks:
HEADER

cat <<BLOCK
  carma_sim_net:
    ipam:
      config:
        - subnet: 172.2.0.0/16
BLOCK
  for i in $(seq $INSTANCES); do
    cat <<BLOCK
  carma_net_$i:
    ipam:
      config:
        - subnet: 172.$(($i+2)).0.0/16
BLOCK
  done
}

carma__start_sim_mode(){
  docker_compose_generater | cat > docker-compose.yml
  echo "Starting CARMA Platform foreground processes..."
  docker run -v docker-compose.yml:/opt/carma/vehicle/config/docker-compose.yml --rm --volumes-from carma-config:ro --entrypoint sh busybox:latest -c \
  'cat /opt/carma/vehicle/config/docker-compose.yml' | \
  docker-compose -f - -p carma up $@
  rm docker-compose.yml
}


carma__start_sim_mode
