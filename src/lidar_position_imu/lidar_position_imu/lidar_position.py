import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class LidarPositionNode(Node):
    def __init__(self):
        # Sets the node name 
        super().__init__('lidar_position_lidar') 

        # Subscribes to the imu data from mid-360
        self.subscriptions = self.create_subscription(
                                Imu, 'livox/imu', self.imu_callback, 10)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()