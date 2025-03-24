#!/bin/bash



# Runs install_spot_ros2.sh for arm or x86
while [ : ]; do
    case "$1" in 
        --arm64)
            echo "arm64"
            sudo chmod +x ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-aarch64.sh && \
	    ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-aarch64.sh
            
            break
            ;;
        *)
            echo "x86"
            sudo chmod +x ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-x86_64.sh && \
	    ${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/install-zed-x86_64.sh
            break
            ;;
    esac
done

cd ${ISAAC_ROS_WS} && \
sudo apt update && \
rosdep update && rosdep install --from-paths src/zed-ros2-wrapper --ignore-src -r -y 
