from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uas_cameras',
            executable='camera1_publisher',
            name='camera1_publisher_node',
            output='screen',
        ),
    ])