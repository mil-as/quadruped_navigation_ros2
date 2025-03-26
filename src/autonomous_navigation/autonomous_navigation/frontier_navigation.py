import rclpy
from rclpy.node import Node
import numpy as np
from scipy.ndimage import label
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Setup subscribers and publishers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setup navigation client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Exploration state
        self.visited_frontiers = set()
        self.map_data = None
        self.robot_position = (0, 0)  # Update from localization
        self.create_timer(5.0, self.explore)

        # Minimum length for frontiers
        self.frontier_min_length = 3  # Default value: adjust as needed

        # Perform initial search
        self.initial_search()

    def find_frontiers(self, map_array):
        """Detect frontiers of any shape between wall cells in the occupancy grid map."""
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

                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
                    nr, nc = r + dr, c + dc

                    if is_valid_cell(nr, nc) and not visited[nr, nc]:
                        if map_array[nr, nc] == -1:  # Unknown cell
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

                    # Frontier is valid if it is bounded by at least two different walls
                    if len(wall_contacts) >= 2 and len(frontier) >= self.frontier_min_length:
                        frontiers.append(frontier)

        self.get_logger().info(f"Found {len(frontiers)} frontiers with length >= {self.frontier_min_length}")
        return frontiers

    def choose_frontier(self, frontiers):
        """Choose the closest frontier to the robot."""
        robot_row, robot_col = self.robot_position
        min_distance, chosen_frontier = float('inf'), None

        for frontier in frontiers:
            center = np.mean(frontier, axis=0).astype(int)
            if tuple(center) in self.visited_frontiers:
                continue
            
            distance = np.hypot(robot_row - center[0], robot_col - center[1])
            if distance < min_distance:
                min_distance, chosen_frontier = distance, center

        if chosen_frontier:
            self.visited_frontiers.add(tuple(chosen_frontier))
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
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
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

    def explore(self):
        """Periodic exploration logic."""
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        self.navigate_to(goal_x, goal_y)

def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
