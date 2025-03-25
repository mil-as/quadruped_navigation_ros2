#!/bin/bash

./nav2_glim_setup.sh
./zed_setup.sh
sudo ./spot_setup.sh

rm -r install/ build/ log/

colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble --parallel-workers 8 

source install/setup.bash

sudo apt update
sudo apt install -y ros-humble-rqt-tf-tree  #better tf viewer

