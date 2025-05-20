# ROS 2 Driven Navigation and Sensor Platform for Quadruped Robots
This repository contains the code for a ROS 2 driven navigation and sensor platform for quadruped robots.
The project was a bachelor project in mechatronics from the Mechatronics Innovation Lab at the University of Agder in spring 2025.

### Repository Contents:
The repository contains documentation and code to control Boston Dynamics Spot from an NVIDIA Jetson AGX Orin running NVIDIA Isaac ROS.
To get data from the environment, a Livox Mid360 3D LiDAR and a Stereolabs Zed 2i stereo RGBD camera are used.
The navigation package used is Navigation 2 (Nav2) alongside SLAM_toolbox for map generation.
A custom package has been developed for autonomous map-making using a frontier-driven approach.

### Abstract from the Finished Paper:
This thesis presents the development of a ROS 2 driven navigation and sensor platform for quadrupeds. The main goal is to develop a platform capable of autonomous navigation and navigating on a pre-made map for use in repeated inspections. To do this, the ROS 2 packages SLAM\_toolbox and Navigation2 are utilized to map and navigate. A custom autonomous exploration package is written for ROS 2 to determine where the robot should walk and explore. The system was tested at the lab and offices at MIL. The results show that it is able to find frontiers and navigate to them autonomously. In addition, pre-made maps can be loaded in and the robot is able to localize and orient itself. These findings demonstrates its ability to navigate autonomously in dynamic environments as well as using pre defined maps.

## Requirements

Ubuntu 22.04 on either ARM64 or x86_64 platforms

## Setup and Installation of Isaac ROS

### Nvidia Jetson Hardware Setup

To set up the Nvidia Jetson follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#jetson-platforms

### x86_64 Hardware Setup

To set up an x86_64 follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html#x86-platforms

### Developer Environment Setup
To set up the developer environment, follow this official guide:

https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

To include the custom Docker file run this command to add .isaac_ros_common-config to the home folder.
```bash
./isaac_ros_common_config_setup.sh
```
or for arm64
```bash
./isaac_ros_common_config_setup.sh --arm64
```


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
#### Spot config file setup 
Rename spot_config_example.yaml to spot_config.yaml and add the username and password to spot.

## Set up when all repositories are already installed
### To set up the packages after the git repositories are installed

For x86_64:
```bash
./complete_setup_x86.sh
```
or for arm64
```bash
./complete_setup_arm64.sh
```
and to also build add ```--build ``` as an argument.

### To set up specific packages

#### For x86_64:

Glim and nav2:
```bash
./nav2_glim_setup.sh
```

Zed:
```bash
./zed_setup.sh
```

Spot:
```bash
sudo ./spot_setup.sh
```

#### For arm64

Glim and nav2:
```bash
./nav2_glim_setup.sh
```

Zed:
```bash
./zed_setup.sh  --arm64
```

Spot:
```bash
sudo ./spot_setup.sh  --arm64
```

## Build packages

To build the package, use:
```bash
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
```
## Launch steps
### Launch custom launch script
In terminal 1 run (Zenoh router):
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

In terminal 2 run (mid360, zed and Spot drivers)
```bash
ros2 launch nav_launch drivers.launch.py
```

#### For online navigation:
In terminal 3 run (nav2 and slamtoolbox)
```bash
ros2 launch nav_launch navigation_online.launch.py
```
##### Launch arguments online:
| Launch argument     | Default value                              |
| ------------------- | ------------------------------------------ |
| nav2_params         | nav_launch/config/nav2_params.yaml         |
| slam_toolbox_params | nav_launch/config/slam_toolbox_params.yaml |
| use_glim            | False                                      |
| glim_config         | nav_launch/config/                         |

#### For offline navigation:
In terminal 3 run (nav2 and Nvidia map localization)
```bash
ros2 launch nav_launch navigation_offline.launch.py
```
##### Launch arguments online:
| Launch argument | Default value                              |
| --------------- | ------------------------------------------ |
| nav2_params     | nav_launch/config/nav2_offline_params.yaml |
| map_yaml        | nav_launch/maps/map.yaml                   |
| use_keepout     | False                                      |
| keepout_yaml    | nav_launch/keepout/keepout_mask.yaml       |

### Launch nodes separate:

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

## Connect to Zenoh router.
Connect to a Zenoh router running on Jetson from an external pc:

```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<ip_jetson>:7447"]'
ros2 run rmw_zenoh_cpp rmw_zenohd
```
