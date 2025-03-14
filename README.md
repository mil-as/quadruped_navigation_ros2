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

To build with the livox_ros_driver, the "ROS_EDITION" and "HUMBLE_ROS" need to be specified:

```bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
```

## Pointcloud to laserscan
Clone the humble branch to /src:

```bash
cd ${ISAAC_ROS_WS}/src
git clone -b humble --single-branch https://github.com/ros-perception/pointcloud_to_laserscan.git
```

Build using the same command as for Livox