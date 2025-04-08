#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from math import sqrt
import numpy as np

MIN_FRONTIER_LENGTH = 20
STATIC_POSITION_TIMER = 30
INITIAL_SEARCH_DURATION = 10

class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Starting frontier exploration")

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/glim_ros/odom', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = False

        self.create_timer(1.0, self.explore)
        self.initial_search()

    def map_callback(self, msg):
        self.map_data = msg

    def pose_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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

    def frontier_detection(self):

        robot_x, robot_y = get_rob
        map_array = self.map_data
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
            print(f"Max distance between points in frontier: {max_distance}")
            return max_distance >= MIN_FRONTIER_LENGTH
        
        while searching:
            if map_queue:
                frontier_point = bfs_frontier_point(map_queue)
            elif not map_queue:
                map_queue.append((robot_x, robot_y))
                frontier_point = bfs_frontier_point(map_queue)
            
            frontier = bfs_find_frontier(frontier_point[0][0], frontier_point[0][1])

            if length_check(frontier):
                searching = False
                return frontier
            else: 
                for r, c in frontier:
                    visited[r, c] = True

    def find_frontier_center(frontier):
        avg_r = int(sum(point[0] for point in frontier) / len(frontier))
        avg_c = int(sum(point[1] for point in frontier) / len(frontier))
        return (avg_r, avg_c)

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
