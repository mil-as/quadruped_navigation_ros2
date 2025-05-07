#!/bin/bash

# Remove incorrect `protoc` version
rm -f /usr/local/bin/protoc
rm -rf /usr/local/include/google/protobuf

# Update packages
apt-get -y update

# Install the correct version of protobuf
apt-get install -y --reinstall protobuf-compiler=3.12.4-1ubuntu7.22.04.1
apt-get install -y --reinstall libprotobuf-dev=3.12.4-1ubuntu7.22.04.1
apt-get install -y --reinstall libprotobuf-lite23

# Install the `proto2ros` package for ROS2
apt-get -y install ros-humble-proto2ros

# Update shell cache
hash -r

# Remove incorrect `libprotobuf` versions from /usr/local/lib
rm -f /usr/local/lib/libprotobuf.so.26.0.0
rm -f /usr/local/lib/libprotobuf-lite.so.26.0.0
rm -f /usr/local/lib/libprotobuf.so
rm -f /usr/local/lib/libprotobuf-lite.so

# Update links for the libraries
ldconfig

# Initialize and update the spot_ros2 submodule
cd /workspaces/isaac_ros-dev/src/spot_ros2
git submodule init
git submodule update

# Runs install_spot_ros2.sh for arm or x86
while [ : ]; do
    case "$1" in 
        --arm64)
            echo "Executing "install_spot_ros2.sh --arm64""
            ./install_spot_ros2.sh --arm64
            break
            ;;
        *)
            echo "Executing "install_spot_ros2.sh""
            ./install_spot_ros2.sh 
            #Installs packages that are missing for x86 for bosdyn
            pip install bosdyn-client \
    	    bosdyn-mission \
            bosdyn-choreography-protos \
            bosdyn-choreography-client 
            break
            ;;
    esac
done

# Install setuptools 61.0.0 to be able to build the package
pip install setuptools==61.0.0

cd ..
# Check if the spot_ros2 directory exists
if [ -d "spot_ros2" ]; then
    # Move the mesh files to spot description 
    cp -r spot_description_config/tassen spot_ros2/spot_description/spot_description/meshes
    cp -r spot_description_config/LIVOX spot_ros2/spot_description/spot_description/meshes
    cp -r spot_description_config/ZED spot_ros2/spot_description/spot_description/meshes
    cp -r spot_description_config/SpotPack spot_ros2/spot_description/spot_description/meshes
    # Replace the spot_macro file to modify it to accept tassen
    cp -r spot_description_config/tassen.urdf.xacro spot_ros2/spot_description/spot_description/urdf
    cp spot_description_config/spot_macro.xacro spot_ros2/spot_description/spot_description/urdf/spot_macro.xacro
fi

