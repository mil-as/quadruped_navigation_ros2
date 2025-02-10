#!/bin/bash

cd ${ISAAC_ROS_WS}/src

# Moves the livox_ros_driver2 to the /src
mv ws_livox/src/livox_ros_driver2 .
# Removes the livox workspace
rm -r ws_livox


# Change the MID360_config with our config file
cp livox_config_launch/MID360_config.json livox_ros_driver2/config/MID360_config.json
# Change the msg_MID360_launch file to publish pointcloud2
cp livox_config_launch/msg_MID360_launch.py livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

#cd livox_ros_driver2/

#if [ -f package.xml ]; then
#    rm package.xml
#fi
#cp -f package_ROS2.xml package.xml
#cp -rf launch_ROS2/ launch/

