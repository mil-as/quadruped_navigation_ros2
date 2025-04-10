#!/bin/bash

sudo apt-get update
# STVL layer for nav2 local costmap
sudo apt-get install -y ros-humble-spatio-temporal-voxel-layer
# required packages for nav 2 and for pointcloud convertion
sudo apt-get install -y ros-humble-pcl-ros # for converting to xyz

# Glim install
sudo apt install -y gpg curl libboost-all-dev libglfw3-dev libmetis-dev
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt-get update
sudo apt install -y libiridescence-dev ros-humble-glim-ros-cuda12.5
sudo ldconfig

# Isaac ROS Map Localization
rosdep update
rosdep install --from-paths src/isaac_ros_map_localization/isaac_ros_occupancy_grid_localizer --ignore-src -y
