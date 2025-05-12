#!/bin/bash

./nav2_glim_setup.sh
./zed_setup.sh --arm64
sudo ./spot_setup.sh --arm64

case "$1" in 
    --build)
        echo "Cleaning up previous builds..."
        rm -r install/ build/ log/
        
        echo "Starting building..."
        colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble 
        ;;
    *)
        ;;
esac
source install/setup.bash