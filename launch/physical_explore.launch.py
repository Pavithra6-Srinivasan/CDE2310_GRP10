from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Explorer Node
        Node(
            package='Autonomous-Explorer-and-Mapper-ros2-nav2',
            executable='explorer',
            name='explorer',
            output='screen'
        ),
    ])
