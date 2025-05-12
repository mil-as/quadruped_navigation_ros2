#!/bin/bash

cd ${ISAAC_ROS_WS}/src

# Check if the ws_livox directory exists
if [ -d "ws_livox" ]; then
    # Move the livox_ros_driver2 to the /src
    mv ws_livox/src/livox_ros_driver2 .
    # Remove the livox workspace
    rm -r ws_livox
fi

# Remove the package.xml file if it exists
if [ -f livox_ros_driver2/package.xml ]; then
    rm livox_ros_driver2/package.xml
fi

# Remove the launch/ folder it exists
if [ -d "livox_ros_driver2/launch" ]; then
    rm -r livox_ros_driver2/launch
fi

# Replace the MID360_config with our config file
cp livox_config_launch/MID360_config.json livox_ros_driver2/config/MID360_config.json
# Replace the msg_MID360_launch file to publish pointcloud2
cp livox_config_launch/msg_MID360_launch.py livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

# Edited files to just send x, y, z and intensity with pointcloud2
#cp livox_config_launch/comm.h livox_ros_driver2/src/comm/comm.h
#cp livox_config_launch/lddc.cpp livox_ros_driver2/src/lddc.cpp

# Create the package.xml file
cp -f livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml
# Create a launch folder with the launch files
cp -rf livox_ros_driver2/launch_ROS2/ livox_ros_driver2/launch/
