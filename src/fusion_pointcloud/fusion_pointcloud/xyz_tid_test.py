import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointCloudToXYZ(Node):
    def __init__(self):
        super().__init__('pointcloud_to_xyz')
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, 10)
        self.rgbd_sub = self.create_subscription(
            PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.rgbd_callback, 10)

        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar/xyz_points', 10)
        self.rgbd_pub = self.create_publisher(PointCloud2, '/rgbd/xyz_points', 10)

    def lidar_callback(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        new_msg = point_cloud2.create_cloud_xyz32(msg.header, points)
        new_msg.header.stamp = self.get_clock().now().to_msg()  # Legger til tidsstempel
        self.lidar_pub.publish(new_msg)

    def rgbd_callback(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        new_msg = point_cloud2.create_cloud_xyz32(msg.header, points)
        new_msg.header.stamp = self.get_clock().now().to_msg()  # Legger til tidsstempel
        self.rgbd_pub.publish(new_msg)

def main():
    rclpy.init()
    node = PointCloudToXYZ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

