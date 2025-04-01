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


## ZED2i setup
Clone the ZED2i repository to the /src folder:

```bash
cd ${ISAAC_ROS_WS}/src
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper
```

Then run zed_setup.sh to make zed script executable and to run the script:

For x86_64:
```bash
cd ${ISAAC_ROS_WS}
./zed_setup.sh
```
Or for arm64:
```bash
cd ${ISAAC_ROS_WS}
./zed_setup.sh --arm64
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

Then run this script to install the required pip packages and uninstall "protoc" and "libprotobuf".
It also moves the updated urdf files from spot_description_config to spot_ros2 folders.

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

In terminal 1 run (Zenoh router):
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

In terminal 2 run (Spot driver):
```bash
ros2 launch spot_driver spot_driver.launch.py config_file:=src/spot_config.yaml
```

In terminal 3 run (mid360 driver):
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

In terminal 4 run (ZED2i driver)
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

In terminal 5 run (Pointcloud to laserscan):
```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```

In terminal 6 run (GLIM):
```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/workspaces/isaac_ros-dev/src/nav_launch/config/
```

In terminal 7 run (Slam toolbox):
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/nav_launch/config/mapper_params_online_async.yaml
```

In terminal 8 run (NAV2):
```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/workspaces/isaac_ros-dev/src/nav_launch/config/nav2_params.yaml
```

## Foxglove
To start the ROS "Foxglove bridge":
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
