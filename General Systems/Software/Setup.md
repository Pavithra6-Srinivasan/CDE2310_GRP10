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

---

## Setup (Laptop)

**Step 1: Create workspace and import custom explorer package from repository**

    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src

**Step 2: Import files from this repository**

Below is the file structure of the custom explorer package on the laptop:
    colcon_ws/
    ├──src/
       ├──Autonomous-Explorer-and-Mapper-ros2-nav2/
          ├── custom_explorer/
              │   ├── __init__.py
              │   ├── **explorer.py**
              |   └── test/
              |       ├── test.copyright.py
              |       ├── test_flake8.py
              |       └── test_pep257.py
              ├── package.xml
              ├── setup.cfg
              └── setup.py

Clone the repository into your ROS 2 workspace:

    cd ~/colcon_ws/src
    git clone https://github.com/Pavithra6-Srinivasan/CDE2310_GRP10

**Step 3: Ensure neccessary dependancies are installed**

You need:
- ROS 2 Humble
- nav2_bringup
- -slam_toolbox
- turtlebot3 packages
- rviz2
- Gazebo (for simulation testing)

    sudo apt update
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-slam-toolbox
    sudo apt install ros-humble-rviz2
    sudo apt install ros-humble-turtlebot3*  # Installs all turtlebot3 packages
    sudo apt install gazebo ros-humble-turtlebot3-gazebo
    sudo apt install ros-humble-turtlebot3-gazebo
    sudo apt install ros-humble-gazebo-ros-pkgs

Install python library:

    pip install numpy

**Step 4: Build workspace**

    cd ~/colcon_ws
    colcon build
    source install/setup.bash

---

## Setup (Raspberry Pi)

**Step 1: Create workspace and launcher_service package**

    mkdir -p ~/ros2_ws/src
    cd ~/launcher_ws/src
    ros2 pkg create --build-type ament_python launcher_service --dependencies rclpy std_srvs

**Step 2: Import *heat_seeker_node.py* and *launcher.py* files are in the Raspberry Pi folder of this repository.**

Below is the file structure of the launcher service package on the Raspberry Pi:

    ros2_ws/
    ├──src/
       ├──launcher_service/
          ├── launcher_service/
              │   ├── __init__.py
              │   ├── **heat_seeker_node.py**
              │   └── **launcher.py**
              ├── package.xml
              └── setup.py

**Step 3: Ensure setup.py is configured properly**

    package_name = 'launcher_service'

**Step 4: Ensure neccessary dependancies are installed**

1. ROS2 for raspberry pi: [https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html]

**Step 5: Install Python libraries on Rpi for AMG8833**

    pip3 install adafruit-circuitpython-amg88xx numpy
    
    sudo apt-get install i2c-tools

Make sure I2C interface enabled:

    sudo raspi-config → Interfacing Options → I2C → Enable

---

## Strting the robot

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

## Acknowledgments

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)
- [explorer.py code](https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2)
