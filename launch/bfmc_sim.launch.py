```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Example perception node
        Node(
            package='bfmc_perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            parameters=[{'camera_topic': '/camera/image_raw'}],
        ),
        # Example control node
        Node(
            package='bfmc_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{'max_speed': 0.6}],
        ),
    ])
