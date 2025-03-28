#!/bin/bash


#STVL layer for nav2 local costmap
sudo apt-get install ros-humble-spatio-temporal-voxel-layer
#required packages for nav 2 and for pointcloud convertion
sudo apt update
#sudo apt install ros-humble-gazebo-ros-pkgs -y Trengs denne?
sudo apt install ros-humble-pcl-ros -y #for converting to xyz


#Glim install
sudo apt update
sudo apt install -y gpg curl libboost-all-dev libglfw3-dev libmetis-dev

curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash


sudo apt update
sudo apt install -y libiridescence-dev ros-humble-glim-ros-cuda12.5
sudo ldconfig


sudo apt-get install ros-humble-nav2-smac-planner



