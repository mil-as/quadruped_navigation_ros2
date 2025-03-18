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

## Livox MID360 setup

Clone the Livox MID360 repository to the /src folder:

```bash
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

Then run MID360_setup.sh to move the MID360_config.json and msg_MID360_launch.py to the livox_ros_driver2 package:

```bash
cd ${ISAAC_ROS_WS}
./MID360_setup.sh
```

## Pointcloud to laserscan
Clone the humble branch to /src:

```bash
cd ${ISAAC_ROS_WS}/src
git clone -b humble --single-branch https://github.com/ros-perception/pointcloud_to_laserscan.git
```

## GLIM
To install GLIM

```bash
./nav2_glim_setup.sh
```

## Spot setup

In the src folder, clone the spot_ros2 repository:
```bash
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/bdaiinstitute/spot_ros2.git
```

Then run this script to install the required pip packages and uninstall "protoc" and "libprotobuf":

For x86_64:
```bash
cd ${ISAAC_ROS_WS}
sudo ./spot_setup.sh
```
Or for arm64:
```bash
cd ${ISAAC_ROS_WS}
sudo ./spot_setup.sh --arm64
```
## Build packages

To build the package, use:
```bash
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
```
## Launch steps

In terminal 1 run (Slam_toolbox, Glim and Pointcloud_to_laserscan):

```bash
ros2 launch nav_launch nav.launch.py 
```
In terminal 2 run (nav2 navigation):

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/workspaces/isaac_ros-dev/src/nav_launch/config/nav2_params.yaml 
```

In terminal 3 run (Spot drivers):

```bash
 ros2 launch spot_driver spot_driver.launch.py config_file:=src/spot_config.yaml 
```


