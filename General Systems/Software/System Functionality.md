# System Functionality

---

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

---

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
  
    map_array = np.array(self.map_data.data).reshape(
        (self.map_data.info.height, self.map_data.info.width))
    
    frontiers = self.find_frontiers(map_array)
    frontiers += self.check_unexplored_areas(map_array)
    
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

---

**heat_seeke_node.py (Raspberry Pi):**

This file has a ROS2 Python node designed to enable the robot to autonomously detct and respond temperatures above threshold values using the AMG8833 thermal infrared sensor. It integrates thermal sensing, robot motion control, server-based commication to support the triggering of a launching mechanism when heat is detected.

**Node Initialisation**

*Refer to Electrical/ for AMG8833 sensor and launcher mechanism setup*

Upon starting, the node initialises I2C bus and sets up communication with AMG8833 sensor. The sensor provides 8x8 array of temperature readings which are used to detect presence of heat sources. 

**Communication**

    self.create_service(Trigger, 'trigger_launcher', self.trigger_launcher_callback)
    self.get_logger().info("trigger_launcher service created.")
    
    self.stop_client = self.create_client(Trigger, 'stop_navigation')
    self.resume_client = self.create_client(Trigger, 'resume_navigation')

The node provides a ROS srrvice names *trigger_launcher*, which is called externally fron *ExplorerNode* to intiate heat seeking and ping pong launching mechanism. The node also acts as a client to 2 external services *stop_navigation* and *resume_navigation*, which pause and resume robot movement respectively. 

**Heat Detection Logic**

    try:
        pixels = self.sensor.pixels
        max_temp = np.max(pixels)
    
        # Print the 8x8 matrix
        self.get_logger().info("AMG8833 Temperature Grid (°C):")
        for row in pixels:
            row_str = " ".join(f"{temp:5.1f}" for temp in row)
            self.get_logger().info(row_str)
    
        # Log the highest temp
        self.get_logger().info(f"Max Temperature: {max_temp:.2f}°C")
    
        # If max temperature exceeds threshold and launcher hasn't been used yet
        if max_temp > self.heat_threshold:
    
            if not self.has_launched:
                self.handle_heat_detected()
        else:
            self.heat_detection_count = 0

Main function runs through loop to check AMG8833 readings. It analyses the sensor's 8x8 matrix to identify the highest temperature detected robots's field of view. If this maximum temperature exceeds a defined threshold and robot has not already launched the ping pong, it triggers head handling sequence.

**Heat Source  Detection**

      pixels = np.array(self.sensor.pixels)
      max_temp = np.max(pixels)
      
      max_index = np.unravel_index(np.argmax(pixels), pixels.shape)
      max_temp = pixels[max_index]
      self.get_logger().info(f"Hottest point: {max_index} at {max_temp:.2f}°C")
      
      while max_index[1] < 3 or max_index[1] > 4:
          twist = Twist()
          twist.linear.x = 0.0
          if max_index[1] < 3:
              twist.angular.z = 0.2  # Turn left
          else:
              twist.angular.z = -0.2  # Turn right
      
          start_time = time.time()
          while time.time() - start_time < 0.2:
              self.cmd_vel_pub.publish(twist)
              time.sleep(0.05)
          self.cmd_vel_pub.publish(Twist())  # Stop
      
          # Update pixels after turn
          pixels = np.array(self.sensor.pixels)
          max_temp = np.max(pixels)
          max_index = np.unravel_index(np.argmax(pixels), pixels.shape)
      
      while max_temp < 33.0:
          twist = Twist()
          twist.angular.z = 0.0
          twist.linear.x = 0.2
          start_time = time.time()
          while time.time() - start_time < 0.2:
              self.cmd_vel_pub.publish(twist)
              time.sleep(0.05)
          self.cmd_vel_pub.publish(Twist())  # Stop
      
          # Update temperature reading
          pixels = np.array(self.sensor.pixels)
          max_temp = np.max(pixels)
          max_index = np.unravel_index(np.argmax(pixels), pixels.shape)
      
      self.get_logger().info("Heat source close. Firing projectile!")
      self.launch_projectile()

When a heat source is detected, the node performs several coordinated actions:
1. Detection: It publishes a message to the /heat_detected topic.
2. Navigation Stopping: The robot requests the stop_navigation service to pause navigation.
3. Rotational Alignment: The robot rotates left or right based on the column position of the hottest pixel until it is centered (columns 3 or 4) to the heat source.
4. Forward Approach: The robot moves forward while monitoring the temperature. Once the temperature reaches a closer threshold, it begins preparation for launching.
5. Projectile Launch: The node calls a local module named launcher to activate launching mechanism.
