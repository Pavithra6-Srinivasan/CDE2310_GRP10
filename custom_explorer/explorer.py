import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.map_data = None
        self.robot_position = (0, 0)  # Grid coordinates (row, col)
        self.robot_pose_world = (0.0, 0.0)  # World coordinates (x, y)

        self.visited_frontiers = set()
        self.visited_world_positions = set()
        self.is_backtracking = False

        # Timer to trigger exploration
        self.timer = self.create_timer(5.0, self.explore)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info_once("Map received")

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.robot_pose_world = (pose.position.x, pose.position.y)

        if self.map_data:
            res = self.map_data.info.resolution
            origin = self.map_data.info.origin.position
            col = int((pose.position.x - origin.x) / res)
            row = int((pose.position.y - origin.y) / res)
            self.robot_position = (row, col)

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            return

        # Dead-end detection: Are there any frontiers adjacent to robot?
        current_frontier_neighbors = [
            f for f in frontiers if self.is_neighbor(f, self.robot_position)
        ]
        self.is_backtracking = len(current_frontier_neighbors) == 0

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            self.get_logger().warning("No suitable frontier to explore")
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        self.visited_world_positions.add((round(goal_x, 2), round(goal_y, 2)))
        self.navigate_to(goal_x, goal_y)

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            world_x = frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
            world_y = frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

            rounded_pos = (round(world_x, 2), round(world_y, 2))
            if not self.is_backtracking and rounded_pos in self.visited_world_positions:
                continue

            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
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

        self.get_logger().info(f"Navigating to goal: x={x:.2f}, y={y:.2f}")
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted by Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info("Navigation succeeded.")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def is_neighbor(self, a, b):
        return abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
