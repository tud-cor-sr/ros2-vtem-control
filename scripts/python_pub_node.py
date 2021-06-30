import numpy as np
import rclpy
from rclpy.node import Node

from vtem_control.msg import FluidPressures
from sensor_msgs.msg import FluidPressure


class VtemControlPublisher(Node):

    def __init__(self):
        super().__init__('vtem_control_publisher')

        self.declare_parameter('vtem_input_pressures_topic', 'vtem_control/input_pressures')

        vtem_input_pressures_topic = self.get_parameter('vtem_input_pressures_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(FluidPressures, vtem_input_pressures_topic, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        pressure_scalar = self.pressure_step(self.i)
        pressure_array = pressure_scalar * 100 * np.ones(shape=(4,))

        msg = FluidPressures()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = []
        for pressure in pressure_array:
            fluid_pressure_msg = FluidPressure()
            fluid_pressure_msg.header.stamp = self.get_clock().now().to_msg()
            fluid_pressure_msg.fluid_pressure = pressure
            msg.data.append(fluid_pressure_msg)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing msg {self.i}: {msg.data}')
        self.i += 1

    def pressure_step(self, i: int) -> int:
        if i % 1500 < 750:
            return 300 # mBar
        else:
            return 450 # mBar


def main(args=None):
    rclpy.init(args=args)

    vtem_control_publisher = VtemControlPublisher()

    rclpy.spin(vtem_control_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vtem_control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()