# inspector-spot
BSc 2025 Project

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

## Spot setup

In the src folder, clone the spot_ros2 repository:
```bash
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/bdaiinstitute/spot_ros2.git
```

Then run this script to install the required pip packages and uninstall "protoc" and "libprotobuf":
```bash
cd ${ISAAC_ROS_WS}

sudo ./spot_setup.sh
or
sudo ./spot_setup.sh --arm64
```

To build the package, use:
```bash
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```