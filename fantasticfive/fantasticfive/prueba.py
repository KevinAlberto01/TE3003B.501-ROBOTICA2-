#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MyNode(Node):
    def __init__(self):
        super().__init__("path")
        self.declare_parameters(namespace = '', parameters = [('float_array', rclpy.Parameter.Type.DOUBLE_ARRAY)])
        
        self.float_array = self.get_parameter('float_array').value # Obtener el valor del parámetro 'float_array'
        self.get_logger().info(f"got time parameter: {self.float_array}")
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pose', 10) # Crear un publicador para el tipo de mensaje Float32MultiArray
        # Configurar un temporizador para llamar al método `timer_callback` periódicamente
        self.timer_period = 1.0  # segundos
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Crear un mensaje Float32MultiArray
        msg = Float32MultiArray()
        msg.data = self.float_array
        
        # Publicar el mensaje
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()