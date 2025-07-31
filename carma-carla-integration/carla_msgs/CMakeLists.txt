cmake_minimum_required(VERSION 3.5)
project(carla_msgs)

find_package(carma_cmake_common REQUIRED)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1
    find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        std_msgs
    )

    add_message_files(DIRECTORY msg
        FILES
            CarlaEgoVehicleControl.msg
            CarlaEgoVehicleInfo.msg
            CarlaEgoVehicleInfoWheel.msg
            CarlaEgoVehicleStatus.msg
    )

    generate_messages(
        DEPENDENCIES
            std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS
            message_runtime
            std_msgs
    )
else() # ROS 2

    find_package(rosidl_default_generators REQUIRED)
    find_package(ament_cmake REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    rosidl_generate_interfaces(
        ${PROJECT_NAME}
            CarlaBoundingBox.msg
            CarlaEgoVehicleControl.msg
            CarlaEgoVehicleStatus.msg
            CarlaEgoVehicleInfoWheel.msg
            CarlaEgoVehicleInfo.msg
            CarlaCollisionEvent.msg
            CarlaLaneInvasionEvent.msg
            CarlaWorldInfo.msg
            CarlaActorInfo.msg
            CarlaActorList.msg
            CarlaControl.msg
            CarlaStatus.msg
            CarlaTrafficLightInfo.msg
            CarlaTrafficLightInfoList.msg
            CarlaTrafficLightStatus.msg
            CarlaTrafficLightStatusList.msg
            CarlaWalkerControl.msg
            CarlaWeatherParameters.msg
        DEPENDENCIES
            std_msgs
            geometry_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()

endif()




