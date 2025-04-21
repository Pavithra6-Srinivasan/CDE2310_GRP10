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
