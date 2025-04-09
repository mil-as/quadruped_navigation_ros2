#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from math import sqrt
import numpy as np

MIN_FRONTIER_LENGTH = 300
INITIAL_SEARCH_DURATION = 10

class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Starting frontier exploration")

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.explored_frontiers = []

        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = False

        self.create_timer(5.0, self.explore)
        self.initial_search()

    def map_callback(self, msg):
        self.map_data = msg

        #self.get_logger().info("Map data received!")

    def pose_callback(self, msg):
        #self.get_logger().info("Pose called back")
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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

    def return_to_home(self):
        if self.home_position is None:
            self.get_logger().error("Start position is not recorded!")
            return

        goal_x, goal_y = self.home_position
        self.navigate_to(goal_x, goal_y)
        self.get_logger().info("Heading home boys!")
        rclpy.shutdown()

    def get_robot_cell(self):
        if not self.map_data:
            return None

        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        robot_row = int((self.robot_position[1] - origin.y) / resolution)
        robot_col = int((self.robot_position[0] - origin.x) / resolution)

        return robot_row, robot_col

    def get_coordinate(self, point):
        goal_x = float(point[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x)
        goal_y = float(point[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y)

        return (goal_x, goal_y)

    def frontier_detection(self, map_array):
        robot_x, robot_y = self.get_robot_cell()
        rows, cols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)
        searching = True
        map_queue = []

        def is_valid_cell(r, c):
            return 0 <= r < rows and 0 <= c < cols

        def free_space_check(r, c):
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                nr, nc = r + dr, c + dc
                if is_valid_cell(nr, nc) and map_array[nr, nc] == 0:
                    return True
            return False

        def bfs_frontier_point(queue):
            frontier_point = []
            while queue:
                r, c = queue.pop(0)
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if is_valid_cell(nr, nc) and map_array[nr, nc] == 0 and not visited[nr, nc]:
                        queue.append((nr, nc))
                        visited[nr, nc] = True
                    elif is_valid_cell(nr, nc) and map_array[nr, nc] == -1 and not visited[nr, nc]:
                        visited[nr, nc] = True
                        frontier_point.append((nr, nc))
                        return frontier_point
            return None

        def bfs_find_frontier(start_r, start_c):
            queue = [(start_r, start_c)]
            visited[start_r, start_c] = True
            frontier = [(start_r, start_c)]

            while queue:
                r, c = queue.pop(0)
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                    nr, nc = r + dr, c + dc
                    if is_valid_cell(nr, nc) and map_array[nr, nc] == -1 and not visited[nr, nc] and free_space_check(nr, nc):
                        queue.append((nr, nc))
                        visited[nr, nc] = True
                        frontier.append((nr, nc))
            return frontier

        def length_check(frontier):
            if not frontier:
                return False
            max_distance = max(
                sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                for p1 in frontier for p2 in frontier
            )
            self.get_logger().info(f"Max distance between points in frontier: {max_distance}")
            return max_distance >= MIN_FRONTIER_LENGTH

        def find_frontier_center(frontier):
            avg_r = int(sum(point[0] for point in frontier) / len(frontier))
            avg_c = int(sum(point[1] for point in frontier) / len(frontier))
            return (avg_r, avg_c)

        while searching:
            if map_queue:
                frontier_point = bfs_frontier_point(map_queue)
            elif not map_queue:
                map_queue.append((robot_x, robot_y))
                visited[robot_x, robot_y] = True
                frontier_point = bfs_frontier_point(map_queue)

            frontier = bfs_find_frontier(frontier_point[0][0], frontier_point[0][1])

            if length_check(frontier):
                searching = False
                self.explored_frontiers.append(frontier)
                frontier_center = find_frontier_center(frontier)
                return frontier_center
            elif map_queue:
                for r, c in frontier:
                    visited[r, c] = True
            elif not map_queue:
                self.get_logger().info(f"All frontiers longer than {MIN_FRONTIER_LENGTH} have been explored")
                self.return_to_home()
                searching = False

    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
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
        except Exception as exept:
            self.get_logger().error(f"Navigation failed: {exept}")
        finally:
            self.is_navigating = False

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if self.is_navigating:
            self.get_logger().info("Navigating")
        else:
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            frontier = self.frontier_detection(map_array)

            frontier_coord = self.get_coordinate(frontier)
            self.navigate_to(frontier_coord[0], frontier_coord[1])
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
