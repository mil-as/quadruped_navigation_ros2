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
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # Launch arguments
    use_keepout_arg = DeclareLaunchArgument(
        'use_keepout',
        default_value = 'False',
        description = 'Use glim to capture pointcloud'
    )

    map_yaml_launch_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(
            get_package_share_directory(
                'nav_launch'), 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )

    keepout_yaml_path = DeclareLaunchArgument(
        'keepout_yaml',
        default_value=os.path.join(
            get_package_share_directory(
                'nav_launch'), 'keepout', 'keepout_mask.yaml'),
        description='Full path to keepout_mask yaml file'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params', default_value=os.path.join(
            get_package_share_directory(
                'nav_launch'), 'config', 'nav2_params_offline.yaml'),
        description='Full path to param file to load')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    # Launch nav2 using nav2 bringup package
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml'),
            'use_sim_time': 'False',
            'params_file': LaunchConfiguration('nav2_params'),
        }.items(),
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/livox/lidar'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 1.2, # 1.2 meter to not see top of doorframe
            'angle_min': -3.14,  # -M_PI/2
            'angle_max': 3.14,  # M_PI/2
            'angle_increment': 0.0075,  # M_PI/360.0
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
        }],
        name='pointcloud_to_laserscan',
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
        parameters=[LaunchConfiguration('map_yaml'), {
            'loc_result_frame': 'map',
            'map_yaml_path': LaunchConfiguration('map_yaml'),
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

    keepout_filter_mask_server = Node(
       package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('keepout_yaml'),
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
            'use_sim_time': False
        }],
        condition = IfCondition(LaunchConfiguration('use_keepout')),
    )

    keepout_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'type': 0,  # type 0 = keepout
            'base': 0.0,
            'multiplier': 1.0,
            'use_sim_time': False
        }],
        condition = IfCondition(LaunchConfiguration('use_keepout')),
    )

    keepout_filter_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }],
        condition = IfCondition(LaunchConfiguration('use_keepout')),
    )

    # Publish base_frame to body 
#    baselink_body_publisher = Node(
#        package='tf2_ros', executable='static_transform_publisher',
#        parameters=[{'use_sim_time': False}],
#        output='screen',
#        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'body'], #-2.8
#    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(use_keepout_arg)
    ld.add_action(map_yaml_launch_arg)
    ld.add_action(keepout_yaml_path)
    ld.add_action(nav2_params_file_arg)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(nav2_launch)
    ld.add_action(occupancy_grid_localizer_container)
    ld.add_action(keepout_filter_mask_server)
    ld.add_action(keepout_filter_info_server)
    ld.add_action(keepout_filter_lifecycle)

#    ld.add_action(baselink_body_publisher)

    return ld
