#!/bin/bash

./nav2_glim_setup.sh
./zed_setup.sh --arm64
./spot_setup.sh --arm64

rm -r install/ build/ log/

colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble --parallel-workers 8 

source install/setup.bash
