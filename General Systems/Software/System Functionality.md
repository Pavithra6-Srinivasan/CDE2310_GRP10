# System Functionality

- **Frontier Detection**: Automatically detects frontiers (unknown areas) in the map.
- **Autonomous Navigation**: Uses Nav2's `NavigateToPose` action to navigate to frontiers.
- **Dynamic Goal Selection**: Chooses the closest unexplored frontier for efficient exploration.
- **ROS 2-Based**: Compatible with ROS 2 (tested on Humble or Foxy distribution).
- **Backtracking**: Backtracks after reaching dead-end in a maze.

## How It Works

1. **Map Subscription**: Subscribes to the `/map` topic to receive occupancy grid maps.
2. **Frontier Detection**: Identifies free cells adjacent to unknown areas as frontiers.
3. **Navigation**: Sends goals to Nav2's `NavigateToPose` action server for autonomous navigation to the closest frontier.
4. **Dynamic Goal Selection**: Continuously updates and selects frontiers during exploration.

## Explorer.py (Laptop):

Script defines a custom ROS2 node, ExplorerNode, responsible for autonomously navigating an unknown environment using *OccupancyGrid* map and *NavigateToPose* action server from Nav2 stack. 
It leverages SLAM for mapping and path planning and incorporates logic for frontier-based exploration, heat detection interruption and safe backtracking.

**Node initialisation**
Upon initialisation of ExplorerNode, it subcribes to 2 topics */map* for receiving the occupancy grid map , and */amcl_pose* for localization updates. It also publishes velovity commands to 
*/cmd_vel* to give directions for  movement and uses *navigate_to_pose action server for navigaion.
The node creates a client for a *trigger_launcher* service, which is used to handle launching mechanism triggered by heat detection from ANG8833. There are 2 other services *stop_navigation* 
and *resume_navigation*, allowing robot to pause and resume navigation when heat is detected and processed. 

**Robot State Management**
The node maintains several internal variables to track the robot's state:

- *map_data*: the current occupancy grid
- *robot_position*: the robot's current cell position in the map
- *robot_pose_world*: the robot’s position in world coordinates
- *visited_frontiers*: grid coordinates of previously explored frontiers
- *visited_world_positions*: world coordinates to avoid revisiting goals
- *heat_handling*: a flag indicating whether launching is being taken care of
- *paused*: whether navigation is currently stopped

**Frontier-Based Navigation Logic**
  
  # Convert map to numpy array
  map_array = np.array(self.map_data.data).reshape(
      (self.map_data.info.height, self.map_data.info.width))
  
  # Detect frontiers and unexplored areas
  frontiers = self.find_frontiers(map_array)
  frontiers += self.check_unexplored_areas(map_array)
  
  # Select frontier and send navigation goal
  chosen_frontier = self.choose_frontier(frontiers)
  if chosen_frontier:
      x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
      y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
      self.navigate_to(x, y)

*explore()* is the main method that is triggered every 5 seconds via a timer. This method converts occupancy grid into a NumPy array and scans for frontiers. The *find_frontiers()* 
function indetifies cells with free space (0) adjacent to unknown space (-1)., makring them as potential goals to navigate to.
  
  for r in range(1, rows - 1):
      for c in range(1, cols - 1):
          if map_array[r, c] == 0:
              neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
              if -1 in neighbors:
                  frontiers.append((r, c))

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
  
The algorithm will then select the frontier to naviagte to using *choose_frontier()*. This function claculates Euclidean distance from robots's current position to each candidate. 
It filers out frontiers already visited ort too close to current position and avoids revisiting unless backtracking is required.

  # Sends a navigation goal to the specified (x, y) location
  goal_msg = PoseStamped()
  goal_msg.header.frame_id = 'map'
  goal_msg.header.stamp = self.get_clock().now().to_msg()
  goal_msg.pose.position.x = x
  goal_msg.pose.position.y = y
  goal_msg.pose.orientation.w = 1.0
  
  nav_goal = NavigateToPose.Goal()
  nav_goal.pose = goal_msg
  
Once a valid frontier is chosen, the robot contructs a NavigateToPose goal and sends it to Nav2 action server using *navigate_to()* method. It monitors responses through 
callbacks such as cancelling goals or handling navigation completion.

**Dead-End Recovery**

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

If all frontiers have been visited or the robot is trapped, the retry_from_furthest_area() function resets exploration from the initial pose or the point closest to the map center, enabling full coverage even in complex environments.
