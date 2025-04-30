import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # Launch arguments
    spot_config_file_args = DeclareLaunchArgument(
        'spot_config',
        default_value = '/workspaces/isaac_ros-dev/src/spot_config.yaml',
        description = 'Path to slam toolbox param file'
    )
    
    # Launch directorys
    spot_driver_launch_dir = os.path.join(
        get_package_share_directory('spot_driver'), 'launch'
    )

    livox_ros_driver2_launch_dir = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'launch_ROS2'
    )

    zed_wrapper_launch_dir = os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch'
    )

    # Create nodes
    # Launch slam toolbox
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spot_driver_launch_dir, 'spot_driver.launch.py')
        ),
        launch_arguments = {
            'config_file': LaunchConfiguration('spot_config')
        }.items(),
    )

    # Launch nav2
    livox_ros_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_ros_driver2_launch_dir, 'msg_MID360_launch.py')),
    )

    zed_wrapper_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_launch_dir, 'zed_camera.launch.py')
        ),
        launch_arguments = {
            'camera_model': 'zed2i'
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(spot_config_file_args)
    ld.add_action(spot_driver_launch)
    ld.add_action(livox_ros_driver2_launch )
    ld.add_action(zed_wrapper_driver_launch)

    return ld
