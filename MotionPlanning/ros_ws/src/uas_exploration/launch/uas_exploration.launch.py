from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'micro-ros-agent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    uas_exploration_node = Node(
        package='uas_exploration',
        executable='uas_exploration',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        uas_exploration_node
    ])
