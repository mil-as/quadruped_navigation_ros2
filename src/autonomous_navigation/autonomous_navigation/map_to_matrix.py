#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid

class MapToArrayNode(Node):
    def __init__(self):
        super().__init__('explorer')

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

        self.array_to_csv(map_array)

    def map_callback(self, msg):

        self.map_data = msg
        self.get_logger().info("Map data received")
    
    def array_to_csv(self, map_array):
        #Convert the map array to a CSV file
        np.savetxt("map_data.csv", map_array, delimiter=",")
        self.get_logger().info("Map data saved to map_data.csv")

def main(args=None):
    rclpy.init(args=args)
    map_to_array_node = MapToArrayNode()

    try:
        rclpy.spin(map_to_array_node)
    except KeyboardInterrupt:
        map_to_array_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    









        