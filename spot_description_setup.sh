#!/bin/bash

cd src

# Check if the spot_ros2 directory exists
if [ -d "spot_ros2" ]; then
    # Move the mesh files to spot description 
    mv spot_description_config/tassen spot_ros2/spot_description/spot_description/meshes
    
    mv spot_description_config/LIVOX spot_ros2/spot_description/spot_description/meshes
    
    mv spot_description_config/ZED spot_ros2/spot_description/spot_description/meshes
    
    mv spot_description_config/tassen.urdf.xacro spot_ros2/spot_description/spot_description/urdf
fi

# Replace the spot_macro file to modify it to accept tassen
cp spot_description_config/spot_macro.xacro spot_ros2/spot_description/spot_description/urdf/spot_macro.xacro


