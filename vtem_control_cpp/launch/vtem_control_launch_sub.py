from launch import LaunchDescription
from launch_ros.actions import Node

#Launch only the node for subscription
def generate_launch_description():
    common_params = {"num_valves": 16, "modbus_node": "192.168.4.3", "modbus_service": "502"}
    return LaunchDescription([
        Node(
            package='vtem_control_cpp',
            namespace='vtem_control',
            executable='input_pressures_sub_node',
            parameters=[
                common_params,
                {"input_pressures_topic": "input_pressures"}
            ]
        ),
    ])