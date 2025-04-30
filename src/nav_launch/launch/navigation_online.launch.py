import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # Launch arguments
    use_glim_arg = DeclareLaunchArgument(
        'use_glim',
        default_value = 'False',
        description = 'Use glim to capture pointcloud'
    )

    glim_params_files_arg = DeclareLaunchArgument(
        'glim_config',
        default_value = os.path.join(
            get_package_share_directory('nav_launch'), 'config'
        ),
        description = 'Path to Glim config folder'
    )

    # Launch paths
    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params',
        default_value = os.path.join(
            get_package_share_directory('nav_launch'), 'config', 'nav2_params.yaml'
        ),
        description = 'Path to nav2 param file'
    )

    slam_toolbox_params_file_args = DeclareLaunchArgument(
        'slam_toolbox_params',
        default_value = os.path.join(
            get_package_share_directory('nav_launch'), 'config', 'slam_toolbox_params.yaml'
        ),
        description = 'Path to slam toolbox param file'
    )

    # Launch directorys
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch'
    )

    # Create nodes
    # Launch pointcloud to laserscan
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
    
    # Launch Glim
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        parameters=[{'config_path': LaunchConfiguration('glim_config')}],
        condition = IfCondition(LaunchConfiguration('use_glim')),
    )
    
    # Launch slam toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')
        ),
        launch_arguments = {
            'slam_params_file': LaunchConfiguration('slam_toolbox_params')
        }.items(),
    )

    # Launch nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params')
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_glim_arg)
    ld.add_action(glim_params_files_arg)
    ld.add_action(nav2_params_file_arg)
    ld.add_action(slam_toolbox_params_file_args)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(glim_node)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_launch)

    return ld
