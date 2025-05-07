#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from math import sqrt
import numpy as np

MIN_FRONTIER_LENGTH = 22
GOAL_TOLERANCE = 1
INITIAL_SEARCH_DURATION = 10
STATIC_POSITION_TIMER = 30

class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Starting frontier exploration")

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_frontier = (0, 0)

        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = None
        self.abandoned_frontiers = []
        self.current_frontier = 0.0
        self.first_counter = 0

        self.initial_search()
        
        self.ref_position = []
        self.abandon = False

        self.create_timer(3.0, self.explore)
        self.create_timer(STATIC_POSITION_TIMER, self.static_position_check)

    def map_callback(self, msg):
        self.map_data = msg

        #self.get_logger().info("Map data received!")

    def pose_callback(self, msg):
        #self.get_logger().info("Pose called back")
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if len(self.ref_position) < 1:
            self.ref_position.append(current_position)
            #self.get_logger().info(f"Starting ref position recorded as {self.ref_position}")

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
    
    def update_ref_position(self):

        self.ref_position.append(self.robot_position)
    
    def static_position_check(self):

        if self.first_counter == 0:
            self.first_counter = 1
            return

        self.get_logger().info("Checking if position is static")

        if not self.ref_position:
            refrence_x, refrence_y = self.robot_position
        else:
            refrence_x, refrence_y = self.ref_position.pop(0)

        trav_dist = np.hypot(refrence_x - self.robot_position[0],
                    refrence_y - self.robot_position[1])
        
        self.get_logger().info(f"Taveled distance is {trav_dist}")

        if trav_dist < 0.5:
            self.abandon = True
            self.get_logger().info("Abandoning frontier")
        else:
            self.abandon = False
           
        self.update_ref_position()
        #self.get_logger().info(f"New ref position is {self.ref_position}")

    def is_at_goal(self):

        goal_tolerance = GOAL_TOLERANCE
        robot_x, robot_y = self.robot_position
        goal_x, goal_y = self.goal_frontier

        if goal_x - goal_tolerance < robot_x < goal_x + goal_tolerance: 
            if goal_y - goal_tolerance < robot_y < goal_y + goal_tolerance:
                return True
            else:
                return False
        return False

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
            return max_distance

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

            if frontier_point is None:
                self.get_logger().info("All frontiers found!")
                self.return_to_home()
                searching = False
                return  

            abandon_check = False

            frontier = bfs_find_frontier(frontier_point[0][0], frontier_point[0][1])
            frontier_length = length_check(frontier)
            
            if abandon_check != False:
                abandon_check = False

            if frontier_length >= MIN_FRONTIER_LENGTH:
                frontier_center = find_frontier_center(frontier)
                if len(self.abandoned_frontiers) >= 1:
                    for length in self.abandoned_frontiers:
                        if length - 1 < frontier_length < length + 1:
                            self.get_logger().info("Abandoned frontier, moving on")
                            abandon_check = True
                            for r, c in frontier:
                                visited[r, c] = True
                            break
                    if not abandon_check:
                        self.current_frontier = frontier_length
                        searching = False
                        return frontier_center

                            
                else:
                    self.current_frontier = frontier_length
                    searching = False 
                    return frontier_center

            else:
                for r, c in frontier:
                    visited[r, c] = True

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
            #return
        
        if goal_handle.accepted:
            self.get_logger().info("Goal accepted")

        goal_handle.get_result_async().add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        
        try:
            result = future.result()
            self.get_logger().info(f"Navigation result: {result}")
        except Exception as exept:
            self.get_logger().error(f"Navigation failed: {exept}")

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        #self.get_logger().info(f"Abandon is {self.abandon}")

        if self.is_navigating:
            if self.abandon:
                self.abandoned_frontiers.append(self.current_frontier)
                self.get_logger().info("These frontiers are abandoned") 
                self.get_logger().info(f"{self.abandoned_frontiers}")
                self.is_navigating = False
                self.abandon = False
                return
            
            elif self.is_at_goal():
                 self.get_logger().info("Arrived at goal")
                 self.is_navigating = False
                 return
            else:
                self.get_logger().info("Navigating")

        else:
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            frontier = self.frontier_detection(map_array)

            frontier_coord = self.get_coordinate(frontier)
            self.goal_frontier = frontier_coord
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
