# CARMA-CARLA Integration Tool
This user guide provides step-by-step user instructions on how to build CARMA-CARLA integration tool on Docker, setup configuration for both CARMA platform and CARMA-CARLA integration tool and run that with CARMA platform.

##  Requirement
-  Docker (19.03+)
-  [Nvidia Docker](https://github.com/NVIDIA/nvidia-docker)
-  [CARMA Platform](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/486178827/Development+Environment+Setup) (4.2.0)
-  [CARLA Simulation](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz) (0.9.10.1)

## Setup
### CARMA-CARLA Integration Setup

1. Getting CARMA-CARLA integration docker image:

Clone carma-carla-integration repository by the command:
```
git clone https://github.com/usdot-fhwa-stol/carma-carla-integration.git
```
Build image from Dockerfile by using following command:

```sh
cd docker && ./build-image.sh 
```

### CARMA Platform Config
CARMA Config for the simulation currently cannot be pulled from docker hub. It requires local docker build for the image.

1. Clone the source code of CARMA config from github:
```sh
git clone https://github.com/usdot-fhwa-stol/carma-config.git
```
2. Go to the directory /carla_integration under CARMA config and build the docker image. Remember the image name
```sh
cd carla_integration/ && ./build-image.sh
```
3. Setup CARMA config of the simulation
```sh
carma config set usdotfhwastol/carma-config:[tag]
```

## Run CARMA-CARLA Integration Tool with CARMA Platform

1. Run CARMA Platform with separated terminal
```
carma start all
```

2.1 Run CARLA server
```
./CarlaUE4.sh
```
2.2 Change the CARLA map to match with CARMA Platform vector map by using the python script provided by CARLA in the path: **`CARLA_0.9.X/PythonAPI/util/config.py`**.
```
python3.7 config.py -m <map>
```
or
```
python2.7 config.py -m <map>
```
depends on your ubuntu setting

3.1 Run carma-carla docker container
```sh
docker run -it --net=host <carma-carla-integration-image-id> /bin/bash
```
3.2 Set the catkin source and CARLA python path
```
export PYTHONPATH=$PYTHONPATH:~/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg && source ~/carma_carla_ws/devel/setup.bash
```
3.3 Launch CARMA-CARLA integration tool
```
roslaunch carma_carla_agent carma_carla_agent.launch synchronous_mode:='true'
```

## Usage Instruction
The usage instruction includes what parameter could be parsed to CARMA-CARLA integration, the description of these parameters and CARMA parameters

### CARMA-CARLA Integration Parameters
| Parameters| **Description**|*Default*|
| ------------------- | ------------------------------------------------------------ |----------|
|host|CARLA server IP address|127.0.0.1|
|port|CARLA server port number|2000|
|town|To specify which scenario for CARLA server to load. The scenario should be matched with executed CARMA Platform vector_map|Town04|
|spawn_point|To specify where to spawn CARLA vehicle|15.4,-90.1,0,0,0,90|
|role_name|Assign the name of the CARLA vehicle. It currently supports the name range from carma_1 to carma_12 |carma_1|
|vehicle_model|To specify what vehicle model should be generated in CARLA server|vehicle.toyota.prius|
|vehicle_length|To specify the length of vehicle|5.00|
|vehicle_width|To specify the width of vehicle|3.00|
|vehicle_wheelbase|To specify the size of wheelbase for the vehicle|2.79|
|speed_Kp| Speed proportional value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.05|
|speed_Ki| Speed integral value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.018|
|speed_Kd| Speed derivative value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.4|
|accel_Kp| Acceleration proportional value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.053|
|accel_Ki| Acceleration integral value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.0|
|accel_Kd| Acceleration derivative value for the vehicle. The current default value was setup for Town04 with vehicle speed limit 20 MPH|0.052|
|init_speed| To specify the initial vehicle speed |5|
|init_acceleration| To specify the initial vehicle acceleration |1|
|init_steering_angle| To specify the initial vehicle steering wheel angle, it range from 0.7(left) to -0.7(right)|0|
|init_jerk| To specify the initial vehicle jerk value|0|
|synchronous_mode|  [CARLA synchronous mode](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#setting-synchronous-mode) |false|
|fixed_delta_seconds| [CARLA fixed delta seconds](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#physics-substepping)|0.05|
|selected_route| Route file name to set to CARMA platform (must exist at /opt/carma/routes) | |
|selected_plugins| Plugin list to set to CARMA platform ||


### CARMA Maps
After CARMA platform installed successfully, the default **`vector_map.osm`** and **`pcd_map.pcd`** are Town02 stored in the folder **`/opt/carma/maps/`**. CARMA platform will load vector map (lanelet) and point cloud map when it has been launched.
The vector map could be converted from open drive (.xodr) by using the [opendrive2lanelet](https://github.com/usdot-fhwa-stol/opendrive2lanelet) tool.
Also, CARLA simulation provides several available Town maps to download from [autoware-contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)

### CARMA Routes
After CARMA platform installed successfully, there is a default Town02 route file located in the folder **`/opt/carma/routes/`**. To customize a route, here is the instruction of [creating a new route file](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1716060161/Creating+a+New+Route+File)
