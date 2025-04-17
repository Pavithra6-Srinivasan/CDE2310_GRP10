# CDE2310 Autonomous Heat-Seeking Robot with Thermal Detection and Projectile Launching

**Objective:** Develop an autonomous robot capable of detecting heat sources using a thermal sensor, aligning itself towards the source, approaching it, and launching a projectile upon reaching a specified temperature threshold.

---

## Features

Hardware Components:

- Raspberry Pi 4 Model B
- OpenCR
- AMG8833 Thermal Camera
- 5N Actuator
- DC Motors with Motor Driver

Software Components:
- ROS 2 (Robot Operating System)
- Python 3
- Custom ROS 2 Nodes: heat_seeker_node.py, explorer.py
- Libraries: numpy, rclpy, adafruit_amg88xx, geometry_msgs, std_msgs

- **Frontier Detection**: Automatically detects frontiers (unknown areas) in the map.
- **Autonomous Navigation**: Uses Nav2's `NavigateToPose` action to navigate to frontiers.
- **Dynamic Goal Selection**: Chooses the closest unexplored frontier for efficient exploration.
- **ROS 2-Based**: Compatible with ROS 2 (tested on Humble or Foxy distribution).
- **Customizable Timer**: Adjust the exploration frequency as needed.

---

## Setup

1. Clone the repository into your ROS 2 workspace:

    cd ~/colcon_ws/src
    git clone https://github.com/Pavithra6-Srinivasan/CDE2310_GRP10
    cd ~/ros2_ws
    colcon build

2. Install dependencies:

    pip install numpy

3. Source the workspace:

    source ~/colcon_ws/install/setup.bash

colcon build --packages-select custom_explorer

1. On RPi - Terminal 1:

    source ~/turtlebot3_ws/install/setup.bash
    ros2 launch turtlebot3_bringup robot.launch.py

2. On RPi- Terminal 2:

    cd ~/ros2_ws
    colcon build --packages-select launcher_service
    source install/setup.bash


    source ~/ros2_ws/install/setup.bash
    ros2 run launcher_service heat_seeker_node

3. On Laptop - Terminal 1:

    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

4. On Laptop - Terminal 2:

    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/home/pavithra/colcon_ws/src/Autonomous-Explorer-and-Mapper-ros2-nav2/nav2_params.yaml

5. On Laptop - Terminal 3:

    cd ~/colcon_ws
    colcon build --packages-select custom_explorer
    source install/setup.bash

    ros2 run custom_explorer explorer

5. On Laptop - Terminal 4:

    ros2 launch nav2_bringup rviz_launch.py

---

## Testing with TurtleBot3

Follow these steps to test the Explorer Node with TurtleBot3 in a Gazebo simulation:

1. Launch the TurtleBot3 world in Gazebo:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. Start the Nav2 stack:

    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

3. Launch SLAM Toolbox for mapping:

    ros2 launch slam_toolbox online_async_launch.py

4. Launch RViz for visualization:

    ros2 launch nav2_bringup rviz_launch.py

5. Run the Explorer Node:

    ros2 run custom_explorer explorer

---

## How It Works

1. **Map Subscription**: Subscribes to the `/map` topic to receive occupancy grid maps.
2. **Frontier Detection**: Identifies free cells adjacent to unknown areas as frontiers.
3. **Navigation**: Sends goals to Nav2's `NavigateToPose` action server for autonomous navigation to the closest frontier.
4. **Dynamic Goal Selection**: Continuously updates and selects frontiers during exploration.

---

## Code Structure

- `explorer.py`: Main node for detecting frontiers and sending navigation goals.
- `requirements.txt`: Python dependencies.
- `README.md`: Project documentation.


## Acknowledgments

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)
- [explorer.py code](https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2)
