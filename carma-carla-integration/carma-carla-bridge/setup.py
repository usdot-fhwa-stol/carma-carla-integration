# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# -*- coding: utf-8 -*-
from setuptools import setup

package_name = 'carma_carla_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        # Launch files and package.xml go to share/
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/carma_carla_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carma',
    maintainer_email='carma@dot.gov',
    description='CARMA-CARLA bridge package for ROS 2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carma_carla_driver_status = carma_carla_bridge.carma_carla_driver_status:main',
            'carma_carla_robot_status = carma_carla_bridge.carma_carla_robot_status:main',
            'carla_to_carma_vehicle_status = carma_carla_bridge.carla_to_carma_vehicle_status:main',
            'carla_to_carma_localization = carma_carla_bridge.carla_to_carma_localization:main',
            'carma_to_carla_sensor_external_objects = carma_carla_bridge.carma_to_carla_sensor_external_objects:main',
            'carma_to_carla_ackermann_cmd = carma_carla_bridge.carma_to_carla_ackermann_cmd:main',
            'carla_to_carma_external_objects = carma_carla_bridge.carla_to_carma_external_objects:main',
            'carma_carla_route = carma_carla_bridge.carma_carla_route:main',
            'carma_carla_plugins = carma_carla_bridge.carma_carla_plugins:main',
            'carma_carla_guidance = carma_carla_bridge.carma_carla_guidance:main',
        ],
    },
)
