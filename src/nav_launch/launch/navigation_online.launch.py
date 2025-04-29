import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory  # Legg til denne importen

def generate_launch_description():
    # Launch arguments
    use_glim_arg = DeclareLaunchArgument(
        'use_glim',
        default_value = 'False',
        description = 'Use glim to capture pointcloud'
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
        get_package_share_directory('nav2_bringup'), 'launch')

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    # Nodes
    glim_node = ComposableNode(
        package='glim_ros',
        plugin='glim_ros::Node',
        name='glim_ros',
        parameters=[{'config_path': get_package_share_directory('nav_launch') + '/config/'}],  # Bruk get_package_share_directory
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    nav2_bringup_launch_dir = os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'
    )

    return LaunchDescription([
    ])
