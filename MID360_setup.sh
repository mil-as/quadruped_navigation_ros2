#!/bin/bash

cd ${ISAAC_ROS_WS}/src

# Move the livox_ros_driver2 to the /src
mv ws_livox/src/livox_ros_driver2 .
# Remove the livox workspace
rm -r ws_livox

# Replace the MID360_config with our config file
cp livox_config_launch/MID360_config.json livox_ros_driver2/config/MID360_config.json
# Update the msg_MID360_launch file to publish pointcloud2
cp livox_config_launch/msg_MID360_launch.py livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

# Remove the package.xml file if there is one
if [ -f livox_config_launch/package.xml ]; then
    rm livox_config_launch/package.xml
fi
# Create the package.xml file
cp -f livox_config_launch/package_ROS2.xml livox_config_launch/package.xml
# Create a launch folder with the launch files
cp -rf livox_config_launch/launch_ROS2/ livox_config_launch/launch/