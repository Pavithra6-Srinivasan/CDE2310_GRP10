import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, CancelResponse
import numpy as np
from tf2_ros import StaticTransformBroadcaster 
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import time
from rclpy.callback_groups import ReentrantCallbackGroup

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Initialized")

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Navigation Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # Service Client to trigger launcher
        self.cli = self.create_client(Trigger, 'trigger_launcher')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trigger service...')

        # Internal State Variables
        self.map_data = None
        self.robot_position = (0, 0)
        self.robot_pose_world = (0.0, 0.0)
        self.visited_frontiers = set()
        self.visited_world_positions = set()
        self.is_backtracking = False
        #self.heat_detected = False
        self.heat_handling = False  # True when launching
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(5.0, self.explore)

        # Stop and Resume Services
        self.cb_group = ReentrantCallbackGroup()
        
        self.stop_srv = self.create_service(
            Trigger, 'stop_navigation', self.handle_stop_navigation, callback_group=self.cb_group)

        self.resume_srv = self.create_service(
            Trigger, 'resume_navigation', self.handle_resume_exploration, callback_group=self.cb_group)

        # More Internal State Variables
        self.active_goal_handle = None
        self.initial_pose = None
        self.launched_positions = set()
        self.launch_distance_threshold = 0.4
        self.paused = False

    # ------------------------ Heat Handling ------------------------

    def handle_stop_navigation(self, request, response):
        # Handles stop request when heat source is detected
        self.get_logger().info("Received request to stop navigation")

        # Cancel active navigation goal
        if self.active_goal_handle is not None:
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self.active_goal_handle = None

        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        response.success = True
        response.message = "Navigation stopped"
        return response

    def handle_resume_exploration(self, request, response):
        # Handles resume request after launcher is fired
        self.get_logger().info("RESUME command received from heat_seeker.")
        self.paused = False
        response.success = True
        response.message = "Navigation resumed."
        return response

    # ------------------------ Navigation ------------------------

    def map_callback(self, msg):
        # store current map
        self.map_data = msg

    def pose_callback(self, msg):
        # Updates robot's position in both world and grid coordinates
        pose = msg.pose.pose
        self.robot_pose_world = (pose.position.x, pose.position.y)

        if not hasattr(self, 'initial_pose'):
            self.initial_pose = (pose.position.x, pose.position.y)

        if self.map_data:
            res = self.map_data.info.resolution
            origin = self.map_data.info.origin.position
            col = int((pose.position.x - origin.x) / res)
            row = int((pose.position.y - origin.y) / res)
            self.robot_position = (row, col)

    # ========================== Main Exploration Logic ==========================

    def explore(self):
        # Main loop that triggers every few seconds to explore
        if self.map_data is None or self.heat_handling:
            return # skip if map is not ready or heat is being handled

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        
        # Detect frontiers and unexplored areas
        frontiers = self.find_frontiers(map_array)
        frontiers += self.check_unexplored_areas(map_array)

        if not frontiers:
            self.get_logger().info("Exploration complete. No more frontiers.")
            self.retry_from_furthest_area()
            return

        # Select frontier and send navigation goal
        chosen_frontier = self.choose_frontier(frontiers)
        if chosen_frontier:
            x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
            y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
            self.navigate_to(x, y)

    def navigate_to(self, x, y):
        # Sends a navigation goal to the specified (x, y) location
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
        # Called when goal is accepted or rejected
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected.")
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)
        
    def navigation_complete_callback(self, future):
        # Called when navigation completes or is interrupted
        if self.heat_handling:
            self.get_logger().info("Navigation interrupted due to heat.")
            self.stop_robot()
            return

        self.get_logger().info("Navigation completed.")
        self.is_backtracking = False
        self.explore()  # continue exploring
    
    def stop_robot(self):
        # Publishes zero velocity to stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped.")

    def _cancel_done_callback(self, future):
        # Logs whether canceling the goal was successful
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Navigation goal canceled")
        else:
            self.get_logger().warning("No goals were canceled")

    # ------------------------ Frontier Detection ------------------------

    def find_frontiers(self, map_array):
        # Finds frontiers — free cells next to unknown cells
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        return frontiers

    def choose_frontier(self, frontiers):
        # Chooses the nearest unexplored frontier
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen = None

        for f in frontiers:
            if f in self.visited_frontiers:
                continue

            world_x = f[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
            world_y = f[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
            pos = (round(world_x, 2), round(world_y, 2))

            if not self.is_backtracking and pos in self.visited_world_positions:
                continue

            dist = np.linalg.norm([robot_row - f[0], robot_col - f[1]])
            if dist < min_distance and dist > 0.3:
                min_distance = dist
                chosen = f

        if chosen:
            self.visited_frontiers.add(chosen)
            x = chosen[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
            y = chosen[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
            self.visited_world_positions.add((round(x, 2), round(y, 2)))

        return chosen

    def check_unexplored_areas(self, map_array):
        # Adds unknown cells (-1) to frontier list (if not visited)
        unexplored = []
        for r in range(map_array.shape[0]):
            for c in range(map_array.shape[1]):
                if map_array[r, c] == -1 and (r, c) not in self.visited_frontiers:
                    unexplored.append((r, c))
        return unexplored

    def retry_from_furthest_area(self):
        # If no frontiers are found, go back to initial or center position
        self.get_logger().info("No more valid frontiers. Restarting exploration from the starting position.")
        
        if hasattr(self, "initial_pose"):
            self.navigate_to(*self.initial_pose)
        elif self.visited_world_positions:
            # Try the closest known position to the center of the maze
            maze_center_x = self.map_data.info.origin.position.x + self.map_data.info.width * self.map_data.info.resolution / 2
            maze_center_y = self.map_data.info.origin.position.y + self.map_data.info.height * self.map_data.info.resolution / 2

            closest = min(
                self.visited_world_positions,
                key=lambda p: np.linalg.norm([maze_center_x - p[0], maze_center_y - p[1]])
            )
            self.get_logger().info(f"Restarting from closest to center: {closest}")
            self.navigate_to(*closest)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
