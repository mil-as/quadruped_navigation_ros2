#!/bin/bash

# Change to the home directory
cd ~ || { echo "Could not change to the home directory"; exit 1; }

# Use echo to create or overwrite the configuration file based on the provided argument
case "$1" in 
    --arm64)
        echo -e "CONFIG_IMAGE_KEY=\"ros2_humble.mine\"\nCONFIG_DOCKER_SEARCH_DIRS=(/mnt/nova_ssd/workspaces/isaac_ros-dev)" > .isaac_ros_common-config
        ;;
    *)
        echo -e "CONFIG_IMAGE_KEY=\"ros2_humble.mine\"\nCONFIG_DOCKER_SEARCH_DIRS=(~/workspaces/isaac_ros-dev)" > .isaac_ros_common-config
        ;;
esac