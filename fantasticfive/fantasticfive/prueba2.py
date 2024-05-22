#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MyNode(Node):
    def __init__(self):
        super().__init__("path")
        self.declare_parameters(namespace = '', parameters = [('float_array', rclpy.Parameter.Type.DOUBLE_ARRAY)])
        self.float_array = self.get_parameter('float_array').value
        self.get_logger().info(f"got time parameter: {self.float_array}")

        self.publisher_ = self.create_publisher(Float32MultiArray, 'path', 10)
        self.Ts = self.get_parameter('float_array').value[1]
        self.publisher_.publish(self.Ts)

        self.get_logger().info(f"got time parameter: {self.Ts}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()