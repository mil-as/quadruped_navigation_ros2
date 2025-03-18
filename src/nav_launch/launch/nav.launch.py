import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'nav_launch'

    glim_config_path = "/workspaces/isaac_ros-dev/src/nav_launch/config/glim"
    slam_toolbox_path = os.path.join(get_package_share_directory(package_name), 'mapper_params_online_async.yaml')
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        parameters=[{'config_path': glim_config_path}],
        output='screen',
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/livox/lidar'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.5,
            'angle_min': -3.14,  # -M_PI/2
            'angle_max': 3.14,  # M_PI/2
            'angle_increment': 0.0075,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': False,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan',
        output='screen',
    )

    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': slam_toolbox_path,
            }.items()
        )

    return LaunchDescription([
        glim_node,
        pointcloud_to_laserscan_node,
        slam_toolbox
    ])
