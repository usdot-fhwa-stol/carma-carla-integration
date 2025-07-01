#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carla_ros2_bridge' 

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'), 
    package_dir={'': 'src'},
    
    data_files=[
        # Install marker file for ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install all .launch.py files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'configs'), glob(os.path.join('configs', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Varner',
    maintainer_email='will.varner@uga.edu',
    description='Ported CARLA ROS 2 bridge for CDASim.',
    license='MIT',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'bridge = carla_ros2_bridge.bridge:main',
            'spawn_vehicle = carla_ros2_bridge.spawn_vehicle:main',
            'ackermann_control = carla_ros2_bridge.ackermann_control.carla_ackermann_control_node:main',
        ],
    },
)