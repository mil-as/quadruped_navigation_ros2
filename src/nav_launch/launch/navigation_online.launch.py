import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
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
        get_package_share_directory('nav_launch'), 'config/'
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


    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch'
    )

    # Create nodes
    
    # Launch pointcloud to laserscan

    # Launch Glim
    glim_node = ComposableNode(
        package='glim_ros',
        plugin='glim_ros::Node',
        name='glim_ros',
        parameters=[{'config_path': LaunchConfiguration('glim_config')}],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition = IfCondition(LaunchConfiguration('use_glim')),
    )

    # Launch slam toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')
        ),
        launch_arguments = {
            'slam_params_file': LaunchConfiguration('slam_toolbox_params')
        }
    )

    # Launch nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments = {
            'params_file': LaunchConfiguration('nav2_params')
        }.items(),
    )
    # Create the launch description and populate
    ld = LaunchDescription([use_glim_arg,])
    ld.add_action(glim_params_files_arg)
    ld.add_action(nav2_params_file_arg)
    ld.add_action(slam_toolbox_params_file_args)
    ld.add_action(pointcloude_to_laser_node)
    ld.add_action(glim_node)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_launch )

    return ld
