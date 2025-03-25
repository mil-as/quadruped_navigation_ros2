from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pointcloud_concat = Node(
        package='pointcloud_concatenate_ros2',
        executable='pointcloud_concat_node',
        name='pc_concat',
        output='screen',
        parameters=[
            {"/clouds": 2},
            {"/hz": 10.0},
            {"/target_frame": "livox_frame"},
        ],
        remappings=[
            ("cloud_in1", "/lidar/xyz_points"),
            ("cloud_in2", "/rgbd/xyz_points"),
            ("cloud_out", "/merged_cloud"),
        ],
    )

    return LaunchDescription([
        pointcloud_concat,
    ])

