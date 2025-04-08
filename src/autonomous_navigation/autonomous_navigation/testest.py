#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import sqrt

MIN_FRONTIER_LENGTH = 20
STATIC_POSITION_TIMER = 30
INITIAL_SEARCH_DURATION = 5

class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Exploration started")

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/glim_ros/odom', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = False
        self.start_time = None
        self.last_position_update_time = None

        self.create_timer(10.0, self.explore)
        self.initial_search()

    def map_callback(self, msg):
        self.map_data = msg

    def pose_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position_update_time is None:
            self.last_position_update_time = self.get_clock().now().seconds_nanoseconds()[0]

        if np.hypot(current_position[0] - self.robot_position[0],
                    current_position[1] - self.robot_position[1]) >= 0.5:
            self.last_position_update_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.robot_position = current_position

        if self.home_position == (0, 0):
            self.home_position = self.robot_position
            self.get_logger().info(f"Home position recorded as: {self.home_position}")

    def initial_search(self):
        self.get_logger().info("Executing initial search")
        search_msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        while self.get_clock().now().seconds_nanoseconds()[0] < current_time + INITIAL_SEARCH_DURATION / 4:
            search_msg.angular.z = 0.5
            self.cmd_vel_publisher.publish(search_msg)

        while self.get_clock().now().seconds_nanoseconds()[0] <= current_time + INITIAL_SEARCH_DURATION / 2:
            search_msg.angular.z = -0.5
            self.cmd_vel_publisher.publish(search_msg)

        while self.get_clock().now().seconds_nanoseconds()[0] <= current_time + INITIAL_SEARCH_DURATION / 4:
            search_msg.angular.z = 0.5
            self.cmd_vel_publisher.publish(search_msg)

        self.cmd_vel_publisher.publish(Twist())
        self.get_logger().info("Completed initial search")

    def get_robot_cell(self):
        if not self.map_data:
            return None

        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        robot_row = int((self.robot_position[1] - origin.y) / resolution)
        robot_col = int((self.robot_position[0] - origin.x) / resolution)

        return robot_row, robot_col

    def identify_frontiers(self):
        if not self.map_data:
            self.get_logger().error("Map data is not available")
            return None

        # Convert map data to a 2D NumPy array
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        robot_x, robot_y = self.get_robot_cell()
        if robot_x is None or robot_y is None:
            self.get_logger().error("Robot position not found in map")
            return None

        def is_valid_cell(r, c):
            return 0 <= r < height and 0 <= c < width

        def free_space_check(r, c):
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                nr, nc = r + dr, c + dc
                if is_valid_cell(nr, nc) and map_array[nr, nc] == 0:
                    return True
            return False

        def bfs_find_frontier(start_r, start_c):
            queue = [(start_r, start_c)]
            frontier = [(start_r, start_c)]

            while queue:
                r, c = queue.pop(0)
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                    nr, nc = r + dr, c + dc
                    if is_valid_cell(nr, nc) and map_array[nr, nc] == -1 and free_space_check(nr, nc):
                        queue.append((nr, nc))
                        frontier.append((nr, nc))
            return frontier

        def length_check(frontier):
            if not frontier:
                return False
            max_distance = max(
                sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                for p1 in frontier for p2 in frontier
            )
            return max_distance >= MIN_FRONTIER_LENGTH

        def find_frontier_center(frontier):
            avg_r = int(sum(point[0] for point in frontier) / len(frontier))
            avg_c = int(sum(point[1] for point in frontier) / len(frontier))
            return (avg_r, avg_c)

        frontier = bfs_find_frontier(robot_x, robot_y)
        if length_check(frontier):
            center = find_frontier_center(frontier)
            return [center]
        else:
            return []

    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to: x={x}, y={y}")
        self.nav_to_goal_client.wait_for_server()
        send_goal_future = self.nav_to_goal_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.is_navigating = False

    def return_to_home(self):
        if self.home_position is None:
            self.get_logger().error("Start position is not recorded!")
            return

        goal_x, goal_y = self.home_position
        self.navigate_to(goal_x, goal_y)
        self.get_logger().info("Heading home!")
        rclpy.shutdown()

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if self.is_navigating:
            elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
            if elapsed_time > STATIC_POSITION_TIMER and self.last_position_update_time is not None:
                if (self.get_clock().now().seconds_nanoseconds()[0] - self.last_position_update_time) > STATIC_POSITION_TIMER:
                    self.get_logger().info("Navigation timeout, stopping navigation")
                    self.start_time = None
                    self.is_navigating = False
            return

        frontier = self.identify_frontiers()
        if not frontier:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.return_to_home()
            return

        goal_x = float(frontier[0][1] * self.map_data.info.resolution + self.map_data.info.origin.position.x)
        goal_y = float(frontier[0][0] * self.map_data.info.resolution + self.map_data.info.origin.position.y)
        self.navigate_to(goal_x, goal_y)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.is_navigating = True

def main(args=None):
    rclpy.init(args=args)
    frontier_explorer_node = FrontierExplorerNode()

    try:
        frontier_explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(frontier_explorer_node)
    except KeyboardInterrupt:
        frontier_explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        frontier_explorer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()