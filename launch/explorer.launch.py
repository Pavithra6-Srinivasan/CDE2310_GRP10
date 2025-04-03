from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Autonomous-Explorer-and-Mapper-ros2-nav2',
            executable='explorer',
            name='explorer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False  # Important for physical robot
            }]
        ),
    ])
