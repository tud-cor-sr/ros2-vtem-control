import numpy as np
import rclpy
from rclpy.node import Node

from vtem_control.msg import InputPressures


class VtemControlPublisher(Node):

    def __init__(self):
        super().__init__('vtem_control_publisher')
        self.publisher_ = self.create_publisher(InputPressures, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        pressure_scalar = self.pressure_step(self.i)
        pressure_array = pressure_scalar * 100 * np.ones(shape=(4,))

        msg = InputPressures()
        msg.data = list(pressure_array)

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