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
            'ackermann_control = carla_ros2_bridge.ackermann_control.carla_ackermann_control_node:main',
        ],
    },
)