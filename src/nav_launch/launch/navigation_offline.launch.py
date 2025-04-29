# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # Launch arguments
    map_yaml_launch_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=os.path.join(
            get_package_share_directory('isaac_ros_occupancy_grid_localizer'), 'maps', 'kart_test.yaml'),
        description='Full path to map yaml file'
    )
    params_file_arg = DeclareLaunchArgument(
        'nav2_param_file', default_value=os.path.join(
            get_package_share_directory(
                'nav_launch'),  'nav2_params.yaml'),
        description='Full path to param file to load')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    # Launch nav2 using nav2 bringup package
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml_path'),
            'use_sim_time': 'False',
            'params_file': LaunchConfiguration('nav2_param_file')
        }.items(),
    )

    # Launch laserscan to flatscan
    laserscan_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
        name='laserscan_to_flatscan'
        # Uncoment to trigger occupansy_grid_localizer every time flatscan is resived
        #remappings=[('/flatscan', '/flatscan_localization')]
    )

    # Launch map localization
    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[LaunchConfiguration('map_yaml_path'), {
            'loc_result_frame': 'map',
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
        }],
        remappings=[('localization_result', '/initialpose')]
    )

    # Add map localisation and lasercan to flatscan nodes in one container
    occupancy_grid_localizer_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='occupancy_grid_localizer_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
            laserscan_to_flatscan_node
        ],
        output='screen'
    )

    # Publish base_frame to body 
#    baselink_body_publisher = Node(
#        package='tf2_ros', executable='static_transform_publisher',
#        parameters=[{'use_sim_time': False}],
#        output='screen',
#        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'body'], #-2.8
#    )

    # Create the launch description and populate
    ld = LaunchDescription([map_yaml_launch_arg,])

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(params_file_arg)
    ld.add_action(nav2_launch)
    ld.add_action(occupancy_grid_localizer_container)
#    ld.add_action(baselink_body_publisher)

    return ld
