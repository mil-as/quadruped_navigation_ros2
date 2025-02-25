import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import numpy as np

class LidarPositionNode(Node):
    def __init__(self):
        # Sets the node name 
        super().__init__('lidar_position_lidar') 

        # Subscribes to the imu data from mid-360
        self.subscriptions = self.create_subscription(
                                Imu, 'livox/imu', self.imu_callback, 10)
        
        # TF2 broadcaster for the lidar frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # For saving the last time stamp
        self.previus_time = None 

    def imu_callback(self, msg):
        # Gets the curant time stamp
        current_time = msg.header.stamp.seck + msg.header.stamp.nanosec * 1e-9

        # Calculated dt
        if self.previus_time is not None:
            dt = current_time - self.previus_time

        self.previus_time = current_time # Updates the previus_time to the curant time

def main(args=None):
    rclpy.init(args=args)
    node = LidarPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()