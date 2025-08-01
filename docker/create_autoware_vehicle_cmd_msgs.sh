#!/bin/bash
set -e

# Variables
WS_DIR=~/carma-carla-integration
SRC_DIR=$WS_DIR/src
PKG_NAME=autoware_vehicle_cmd_msgs
AUTOWARE_REPO=https://github.com/usdot-fhwa-stol/autoware.ai.git
AUTOWARE_BRANCH=carma-develop

echo "### Creating minimal ROS 2 package for VehicleCmd ###"

# Clone autoware.ai to extract messages
if [ ! -d "$HOME/autoware_msgs" ]; then
  echo "Cloning autoware.ai repository..."
  git clone --depth 1 --branch $AUTOWARE_BRANCH $AUTOWARE_REPO ~/autoware_msgs
fi

# Create ROS 2 package
mkdir -p $SRC_DIR
cd $SRC_DIR

if [ -d "$PKG_NAME" ]; then
  echo "Removing old package $PKG_NAME..."
  rm -rf $PKG_NAME
fi

echo "Creating package $PKG_NAME..."
ros2 pkg create $PKG_NAME --build-type ament_cmake --dependencies builtin_interfaces geometry_msgs

# Copy only required .msg files
echo "Copying VehicleCmd and its dependencies..."
mkdir -p $PKG_NAME/msg
cp ~/autoware_msgs/messages/autoware_msgs/msg/{VehicleCmd.msg,SteerCmd.msg,AccelCmd.msg,BrakeCmd.msg,LampCmd.msg,ControlCommand.msg} \
   $PKG_NAME/msg/

# Update CMakeLists.txt
CMAKE_FILE=$PKG_NAME/CMakeLists.txt
echo "Updating CMakeLists.txt..."
cat <<EOL >> $CMAKE_FILE

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/VehicleCmd.msg"
  "msg/SteerCmd.msg"
  "msg/AccelCmd.msg"
  "msg/BrakeCmd.msg"
  "msg/LampCmd.msg"
  "msg/ControlCommand.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
EOL

# Update package.xml
PKG_XML=$PKG_NAME/package.xml
echo "Updating package.xml..."
cat <<EOL > $PKG_XML
<?xml version="1.0"?>
<package format="3">
  <name>${PKG_NAME}</name>
  <version>0.0.1</version>
  <description>Minimal ROS 2 package for VehicleCmd and dependencies.</description>
  <maintainer email="black@email.com">BlankName</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rosidl_default_generators</depend>
  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
EOL

# Build package
echo "Building $PKG_NAME..."
cd $WS_DIR
colcon build --symlink-install --packages-select $PKG_NAME

echo "Sourcing workspace..."
source $WS_DIR/install/setup.bash

echo "### Custom autoware_msgs built successfully! ###"
ros2 interface show autoware_vehicle_cmd_msgs/msg/VehicleCmd
