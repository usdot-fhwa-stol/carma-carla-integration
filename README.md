# CARMA-CARLA Integration Tool
This user guide provides step-by-step user instructions on how to build CARMA-CARLA integration tool on Docker, setup configuration for both CARMA platform and CARMA-CARLA integration tool and run that with CARMA platform.

##  Requirement
-  Docker (19.03+)
-  [Nvidia Docker](https://github.com/NVIDIA/nvidia-docker)
-  [CARMA Platform](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/486178827/Development+Environment+Setup) (3.9.0)
-  [CARLA Simulation](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz) (0.9.10.1)

## Setup
### CARMA-CARLA Integration Setup

1. Getting CARMA-CARLA integration docker mage:

There are two ways to get CARMA-CARLA docker image. You can either choose to pull the co-simulation tool docker image from DockerHub or build the image by yourself.

```
git clone https://github.com/usdot-fhwa-stol/carma-carla-integration.git
```
Option 1. Build image from Dockerfile by using following command:

```sh
cd docker && ./build-image.sh -v [version]
```

Option 2. Pull image from DockerHub by using following command:

```sh
docker pull usdotfhwastol/carma-carla-integration:[tag]
```

The tag information could be found from [usdotfhwastol Dockerhub](https://hub.docker.com/repository/docker/usdotfhwastol/carma-carla-integration)

2. Edit configuration files

#### Vehicle Configuration
Vehicle configuration is a json file. The number of CARMA instances in a run will be determined by how many elements in the ``Instances`` json list.

#### Global Config
Global configuration is a json file. It includes every docker image tag and CARLA map information which will be pared when tool being launched.

***Note: Both of the configuration files could be edited under the path ``<path-to-carma-carla-integration>/config`` before first run, then it will be copied and only being read from ``/opt/carma-simulation``***



## Run CARMA-CARLA Integration Tool with CARMA Platform
Run carma_start_sim_mode.sh
```sh
cd <path-to-carma-carla-integration>/docker && ./carma_start_sim_mode.sh
```



## Usage Instruction
The usage instruction includes what parameter could be parsed to CARMA-CARLA integration, the description of these parameters and CARMA parameters

### CARMA-CARLA Integration Parameters
| Parameters| **Description**|*Default*|
| ------------------- | ------------------------------------------------------------ |----------|
|spawn_point|To specify where to spawn CARLA vehicle|N/A|
|role_name|Assign the name of the CARLA vehicle. It currently only supports the name range from hero0 to hero9 and ego_vehicle|ego_vehicle|
|selected_route|To specify route file for CARMA platform. Route file could be found in the path ``/opt/carma/route/`` |N/A|
|selected_plugin|To specify plugins for CARMA platform. Required plugins are ``RouteFollowingPlugin``, ``InLaneCruisingPlugin``, ``StopAndWaitPlugin``, ``Pure Pursuit``|N/A|
|speed_Kp| Speed proportional value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.05|
|speed_Ki| Speed integral value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.018|
|speed_Kd| Speed derivative value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.4|
|accel_Kp| Acceleration proportional value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.053|
|accel_Ki| Acceleration integral value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.0|
|accel_Kd| Acceleration derivative value for the vehicle. The current default value was setup for Town02 with vehicle speed limit 20 MPH|0.052|
|init_speed| To specify the initial vehicle speed |5|
|init_acceleration| To specify the initial vehicle acceleration |1|
|init_steering_angle| To specify the initial vehicle steering wheel angle, it range from 0.7(left) to -0.7(right)|0|
|init_jerk| To specify the initial vehicle jerk value|0|


### CARMA Maps
After CARMA platform installed successfully, the default **`vector_map.osm`** and **`pcd_map.pcd`** are Town02 stored in the folder **`/opt/carma/maps/`**. CARMA platform will load vector map (lanelet) and point cloud map when it has been launched.
The vector map could be converted from open drive (.xodr) by using the [opendrive2lanelet](https://github.com/usdot-fhwa-stol/opendrive2lanelet) tool.
Also, CARLA simulation provides several available Town maps to download from [autoware-contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)

### CARMA Routes
After CARMA platform installed successfully, there is a default Town02 route file located in the folder **`/opt/carma/route/`**. To customize a route, here is the instruction of [creating a new route file](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1716060161/Creating+a+New+Route+File)
