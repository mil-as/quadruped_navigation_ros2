#! /usr/bin/env python

import numpy as np
from collections import deque
from math import sqrt
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

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

        self.explored_frontiers = set()
        self.abandoned_frontiers = set()
        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = False
        self.start_time = None
        self.last_position_update_time = None
        self.free_cells = set()  # Cache for free cells
        self.frontiers = []  # Cache for frontiers

        self.create_timer(10.0, self.explore)
        self.initial_search()

    def map_callback(self, msg):
        self.map_data = msg
        self.cache_free_cells()

    def cache_free_cells(self):
        """Cache all free cells (value 0) from the map."""
        if self.map_data is None:
            return

        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        self.free_cells = {(r, c) for r in range(map_array.shape[0]) for c in range(map_array.shape[1]) if map_array[r, c] == 0}
        self.get_logger().info(f"Cached {len(self.free_cells)} free cells")

    def pose_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if self.last_position_update_time is None:
            self.last_position_update_time = now

        if np.hypot(current_position[0] - self.robot_position[0],
                    current_position[1] - self.robot_position[1]) >= 0.5:
            self.last_position_update_time = now

        self.robot_position = current_position

        if self.home_position == (0, 0):
            self.home_position = self.robot_position
            self.get_logger().info(f"Home position recorded as: {self.home_position}")

    def initial_search(self):
        self.get_logger().info("Executing initial search")
        search_msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        for duration, angular_z in [(INITIAL_SEARCH_DURATION / 4, 0.5),
                                    (INITIAL_SEARCH_DURATION / 2, -0.5),
                                    (INITIAL_SEARCH_DURATION / 4, 0.5)]:
            end_time = current_time + duration
            while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
                search_msg.angular.z = angular_z
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

    def identify_frontiers(self, map_array):
        rows, cols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)

        def is_valid_cell(r, c):
            return 0 <= r < rows and 0 <= c < cols

        def bfs_find_frontier(start_r, start_c):
            queue = deque([(start_r, start_c)])
            visited[start_r, start_c] = True
            potential_frontier = []

            while queue:
                r, c = queue.popleft()
                if map_array[r, c] == 0:
                    potential_frontier.append((r, c))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if is_valid_cell(nr, nc) and not visited[nr, nc] and map_array[nr, nc] == 0:
                        queue.append((nr, nc))
                        visited[nr, nc] = True

            return [(r, c) for r, c in potential_frontier if any(
                is_valid_cell(r + dr, c + dc) and map_array[r + dr, c + dc] == -1
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)])]

        frontier_points = []
        for cell in self.free_cells:
            if not visited[cell[0], cell[1]]:
                frontier_points.extend(bfs_find_frontier(cell[0], cell[1]))

        return self.group_frontiers(frontier_points)

    def group_frontiers(self, frontier_points):
        """Group frontier points into clusters."""
        grouped_frontiers = []
        frontier_set = set(frontier_points)

        while frontier_set:
            start_point = frontier_set.pop()
            queue = deque([start_point])
            current_frontier = [start_point]

            while queue:
                r, c = queue.popleft()
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    neighbor = (r + dr, c + dc)
                    if neighbor in frontier_set:
                        queue.append(neighbor)
                        current_frontier.append(neighbor)
                        frontier_set.remove(neighbor)

            if len(current_frontier) >= 2:
                max_distance = max(
                    sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                    for p1 in current_frontier for p2 in current_frontier)
                if max_distance >= MIN_FRONTIER_LENGTH:
                    grouped_frontiers.append(current_frontier)

        return [(int(sum(p[0] for p in frontier) / len(frontier)),
                 int(sum(p[1] for p in frontier) / len(frontier))) for frontier in grouped_frontiers]

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if self.is_navigating:
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - self.start_time > STATIC_POSITION_TIMER and self.last_position_update_time is not None:
                if now - self.last_position_update_time > STATIC_POSITION_TIMER:
                    self.abandoned_frontiers.add(tuple(self.current_frontier))
                    self.get_logger().info(f"Frontier {self.current_frontier} abandoned due to lack of progress")
                    self.start_time = None
                    self.current_frontier = None
                    self.is_navigating = False
            return

        if not self.frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.return_to_home()
            return

        chosen_frontier = self.choose_frontier(self.frontiers)
        if chosen_frontier is None:
            self.return_to_home()
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        self.navigate_to(goal_x, goal_y)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_frontier = chosen_frontier
        self.is_navigating = True

    def choose_frontier(self, frontier_centers):
        robot_row, robot_col = self.get_robot_cell()
        if robot_row is None or robot_col is None:
            self.get_logger().info("Robot position not available")
            return None

        chosen_frontier = min(
            (center for center in frontier_centers if tuple(center) not in self.abandoned_frontiers),
            key=lambda center: np.hypot(robot_row - center[0], robot_col - center[1]),
            default=None
        )

        if chosen_frontier:
            self.explored_frontiers.add(tuple(chosen_frontier))
            self.get_logger().info(f"Chosen frontier center: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

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
            self.update_frontiers_after_navigation()
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.is_navigating = False

    def update_frontiers_after_navigation(self):
        """Update frontiers after reaching a target."""
        if self.map_data is not None and self.free_cells:
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            self.frontiers = self.identify_frontiers(map_array)
            self.get_logger().info(f"Updated frontiers: {len(self.frontiers)} found")

    def return_to_home(self):
        if self.home_position is None:
            self.get_logger().error("Start position is not recorded!")
            return

        self.navigate_to(*self.home_position)
        self.get_logger().info("Heading home boys!")
        rclpy.shutdown()

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