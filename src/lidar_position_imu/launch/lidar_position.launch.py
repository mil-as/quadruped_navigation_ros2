from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   lidar_position_node = Node(
      package='lidar_position_imu',
      executable='lidar_position',
      name='lidar_position_from_imu',
      output='screen',
   )
   
   return LaunchDescription([
    lidar_position_node, 
   ])