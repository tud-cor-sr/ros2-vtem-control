from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vtem_control',
            namespace='vtem_control',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='vtem_control',
            namespace='vtem_control',
            executable='output_pressures_pub_node',
            parameters=[
                {"output_pressures_topic": "test"}
            ]
        ),
    ])