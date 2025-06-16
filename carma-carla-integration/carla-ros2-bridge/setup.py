from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'carla_ros2_bridge' 

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'), 
    package_dir={'': 'src'},
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Varner',
    maintainer_email='will.varner@uga.edu',
    description='Ported CARLA ROS 2 bridge for CDASim.',
    license='MIT',
    
    entry_points={
        'console_scripts': [
            # This base PR only provides the main bridge executable
            'bridge = carla_ros2_bridge.bridge:main',
        ],
    },
)