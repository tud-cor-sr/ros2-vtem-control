from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = {"modbus_node": "192.168.4.3", "modbus_service": "502"}
    return LaunchDescription([
        Node(
            package='vtem_control',
            namespace='vtem_control',
            executable='input_pressures_sub_node',
            parameters=[
                common_params,
                {"input_pressures_topic": "input_pressures"}
            ]
        ),
        Node(
            package='vtem_control',
            namespace='vtem_control',
            executable='output_pressures_pub_node',
            parameters=[
                common_params,
                {"output_pressures_topic": "output_pressures", "pub_freq": 50.}
            ]
        ),
    ])