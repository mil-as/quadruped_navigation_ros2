# inspector-spot

BSc 2025 Project


## Requirements

Ubuntu 22.04 on either ARM64 or x86_64 platforms

## Setup and Installation of Isaac ROS

### Nvidia Jetson Hardware Setup

To  up the Nvidia Jetson follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms

Then set up VPI for Jetson:

https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_vpi.html

### x86_64 Hardware Setup

To set up a x86_64 follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#x86-platforms

### Developer Environment Setup
To set up the developer environment setup follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

## Isaac ROS Dev Container

To start, clone "Isaac_ros_common" to the /src folder:
```bash
cd ${ISAAC_ROS_WS}/src && \
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```
To start the container, run:
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```
To open a new terminal in the same container, run the same command.

## Foxglove
To start the ROS "Foxglove bridge":
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
