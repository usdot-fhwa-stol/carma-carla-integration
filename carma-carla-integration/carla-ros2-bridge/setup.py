import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'carla_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'rclpy', 'numpy', 'transforms3d'],
    zip_safe=True,
    maintainer='Will Varner',
    maintainer_email='will.varner@uga.edu',
    description='A custom ROS 2 bridge for CARLA 0.10.0 to publish odometry, object, vehicle status, and vehicle info for UGA MSC Lab.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher_node = carla_ros2_bridge.odometry_publisher_node:main',
            'objects_publisher_node = carla_ros2_bridge.objects_publisher_node:main',
            'vehicle_status_publisher_node = carla_ros2_bridge.vehicle_status_publisher_node:main',
            'vehicle_info_publisher_node = carla_ros2_bridge.vehicle_info_publisher_node:main',
        ],
    },
)