#! /usr/bin/env python3

#Importing all nessesary modules and libraried

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

MIN_FRONTIER_LENGTH = 0.6
STATIC_POSITION_TIMER = 10
INITIAL_SEARCH_DURATION = 5

#A class that inherites from the node class
#This class is used to controll the autonomous navigation

class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Frontier exploration started")

        #The nessesary setup for publishers and subscribers

        self.map_subscribe = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/glim_ros/odom', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #A action client for navigation

        self.nav_to_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        #Keeping track of what frontiers have been explored,
        #and what frontiers are unreachable

        self.explored_frontiers = set()
        self.abandoned_frontiers = set()

        #Exploration parameters

        self.map_data = None
        self.robot_position = (0, 0)
        self.home_position = (0, 0)
        self.is_navigating = False

        #Interval timer for explore method

        self.create_timer(5.0, self.explore)

        #Making a initial half turn for search where lidar cant see initially

        self.start_time = None 
        self.last_position_update_time = None 


        self.initial_search()

    def map_callback(self, msg):

        #Map data processing

        self.map_data = msg
        self.get_logger().info("Map data received")

    def pose_callback(self, msg):

        #Odometry data processing

        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        #self.get_logger().info(f"Updated robot position: {current_position}")

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

        while self.get_clock().now().seconds_nanoseconds()[0] < current_time + INITIAL_SEARCH_DURATION/2:
            search_msg.angular.z = 0.5
            self.cmd_vel_publisher.publish(search_msg)
        
        while self.get_clock().now().seconds_nanoseconds()[0] <= current_time + INITIAL_SEARCH_DURATION:
            search_msg.angular.z = -0.5
            self.cmd_vel_publisher.publish(search_msg)

        search_stop_msg = Twist()
        self.cmd_vel_publisher.publish(search_stop_msg)
        self.get_logger().info("Completed initial search")

    def identify_frontiers(self, map_array):

        rows, cols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)
        frontiers = []

        def is_valid_cell(r, c):
            return 0 <= r < rows and 0 <= c < cols

        def is_wall(r, c):
            return map_array[r, c] == 100

        def bfs_find_frontier(start_r, start_c):
            queue = [(start_r, start_c)]
            visited[start_r, start_c] = True
            frontier = [(start_r, start_c)]
            wall_contacts = []

            while queue:
                r, c = queue.pop(0)

                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), 
                               (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                    nr, nc = r + dr, c + dc

                    if is_valid_cell(nr, nc) and not visited[nr, nc]:
                        if map_array[nr, nc] == -1:
                            queue.append((nr, nc))
                            visited[nr, nc] = True
                            frontier.append((nr, nc))
                        elif is_wall(nr, nc):
                            wall_contacts.append((nr, nc))

            return frontier, wall_contacts

        # Explore each unknown cell and find frontiers
        for r in range(rows):
            for c in range(cols):
                if map_array[r, c] == -1 and not visited[r, c]:
                    frontier, wall_contacts = bfs_find_frontier(r, c)

                    if len(wall_contacts) >= 2 and len(frontier) >= MIN_FRONTIER_LENGTH:
                        frontiers.append(frontier)

        self.get_logger().info(f"Found {len(frontiers)} frontiers with length >= {MIN_FRONTIER_LENGTH}")
        return frontiers
    
    def choose_frontier(self, frontiers):

        robot_row = (self.robot_position[1]-self.map_data.info.origin.position.x)/self.map_data.info.resolution
        robot_col = (self.robot_position[0]-self.map_data.info.origin.position.y)/self.map_data.info.resolution
        min_distance, chosen_frontier = float('inf'), None

        for frontier in frontiers:
            center = np.mean(frontier, axis=0).astype(int)
            if tuple(center) in self.explored_frontiers or tuple(center) in self.abandoned_frontiers:
                continue
            
            distance = np.hypot(robot_row - center[0], robot_col - center[1])
            if distance < min_distance:
                min_distance, chosen_frontier = distance, center

        if chosen_frontier is not None:
            self.explored_frontiers.add(tuple(chosen_frontier))
            self.get_logger().info(f"Chosen frontier center: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier
    
    def navigate_to(self, x, y):
        """Send navigation goal to Nav2."""
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
        """Handle goal response and attach callback to the result."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """Handle navigation completion result."""
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.is_navigating = False

    def return_to_home(self):
        """Navigate back to start position and shut down"""
        self.get_logger().info("Returning to start position...")
        
        if self.home_position is None:
            self.get_logger().error("Start position is not recorded!")
            return
        
        goal_x = self.home_position[0]
        goal_y = self.home_position[1]
        self.navigate_to(goal_x, goal_y)

        # Shut down after reaching home
        self.get_logger().info("Reached start position, shutting down.")
        rclpy.shutdown()

    def explore(self):
        """Periodic exploration logic."""
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return
        
        if self.is_navigating:
            self.get_logger().info("Already navigating, skipping exploration")
            return
        
        if self.start_time is not None and self.get_clock().now().seconds_nanoseconds()[0] - self.start_time > STATIC_POSITION_TIMER:
            if self.last_position_update_time is not None and (self.get_clock().now().seconds_nanoseconds()[0] - self.last_position_update_time) > STATIC_POSITION_TIMER:
                self.abandoned_frontiers.add(tuple(self.current_frontier))
                self.get_logger().info(f"Frontier {self.current_frontier} abandoned due to lack of progress")
                self.start_time = None 
                self.current_frontier = None

        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        frontiers = self.identify_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.return_to_home()
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if chosen_frontier is None:
            self.get_logger().warning("No frontiers to explore")
            self.return_to_home()
            return

        goal_x = float(chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x)
        goal_y = float(chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y)
        self.navigate_to(goal_x, goal_y)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Oppdaterer starttimen når vi navigerer til en ny frontier
        self.current_frontier = chosen_frontier  # Oppdaterer nåværende frontier
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

    









        