cmake_minimum_required(VERSION 3.8)
project(carla_ros2_bridge_cdasim) # Or whatever you named your package in package.xml

# Find dependencies.
# For a pure Python package, ament_cmake is the main one.
find_package(ament_cmake REQUIRED)
# For ROS 2 Humble and newer, find Python 3 directly:
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# This macro handles the installation of Python packages
# based on setup.py and the ament_python build type in package.xml.
ament_python_install_package()

# This allows the package to be found by other ROS 2 packages
# and registers tests if you have them.
ament_package()