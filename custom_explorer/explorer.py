import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from tf2_ros import StaticTransformBroadcaster 
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import time


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

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_rate = 1.0  # Set your desired publish rate (Hz)
        self.last_publish_time = self.get_clock().now()

        self.heat_detected = False
        self.heat_sub = self.create_subscription(Bool, '/heat_detected', self.heat_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # If not already there

        self.cli = self.create_client(Trigger, 'launch_projectile')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for launcher service...')

    def pause_and_launch(self):
        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        time.sleep(1.0)  # Give it a moment to stop

        # Call the launcher service
        self.call_launcher()

        # Resume exploration after launching
        self.heat_detected = False
        self.get_logger().info("Resuming exploration.")

    def heat_callback(self, msg):
        if msg.data and not self.heat_detected:
            self.heat_detected = True
            self.get_logger().info("Heat detected. Pausing exploration and launching.")
            self.pause_and_launch()

    def call_launcher(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.launcher_response_callback)

    def launcher_response_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Launcher activated')
            else:
                self.get_logger().error('Launcher failed: ' + result.message)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def publish_navigation_goal(self, x, y):
        current_time = self.get_clock().now()
        if (current_time - self.last_publish_time).seconds < (1 / self.publish_rate):
            return

        # Proceed with publishing the goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = current_time.to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        self.nav_to_pose_client.send_goal_async(nav_goal)
        self.last_publish_time = current_time

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
        unexplored_areas = self.check_unexplored_areas(map_array)
        if unexplored_areas:
            self.get_logger().info(f"Found unexplored areas. Re-initiating exploration.")
            frontiers.extend(unexplored_areas)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.retry_from_furthest_area()
            return

        # Choose the closest frontier
        chosen_frontier = self.choose_frontier(frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No suitable frontier to explore")
            self.retry_from_furthest_area()
            return

        world_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        world_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        
        self.navigate_to(world_x, world_y)

    def retry_from_furthest_area(self):
        if not self.visited_world_positions:
            self.get_logger().warning("No visited positions to go back to.")
            return

        # Find the furthest position from the robot
        robot_x, robot_y = self.robot_pose_world
        furthest_pos = max(self.visited_world_positions, key=lambda pos: np.sqrt((robot_x - pos[0])**2 + (robot_y - pos[1])**2))
        self.get_logger().info(f"Retrying from the furthest position: {furthest_pos}")
        self.navigate_to(*furthest_pos)

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape

        # Identify potential frontiers
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown (frontiers)
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:  # If there's an unknown cell, this is a frontier
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
            # Add condition to skip close frontiers
            if distance < 0.3:
                continue
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
            
        return chosen_frontier

    def check_unexplored_areas(self, map_array):
        """
        Check for areas that have not been explored and return them as new frontiers.
        """
        unexplored_areas = []
        rows, cols = map_array.shape

        # Iterate through the entire map to find unexplored regions
        for r in range(rows):
            for c in range(cols):
                if map_array[r, c] == -1 and (r, c) not in self.visited_frontiers:
                    unexplored_areas.append((r, c))

        return unexplored_areas

    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        # Add transform tolerance
        nav_goal.transform_tolerance.sec = 1
        nav_goal.transform_tolerance.nanosec = 0

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
