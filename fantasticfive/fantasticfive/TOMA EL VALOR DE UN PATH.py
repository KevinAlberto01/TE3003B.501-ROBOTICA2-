import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FloatArrayPublisher(Node):

    def __init__(self):
        super().__init__('float_array_publisher')
        
        # Declarar el parámetro 'float_array' como una lista de doubles
        self.declare_parameters(namespace='', parameters=[('float_array', rclpy.Parameter.Type.DOUBLE_ARRAY)])
        # Obtener el valor del parámetro 'float_array'
        self.float_array = self.get_parameter('float_array').value
        self.get_logger().info(f"Obtained parameter: {self.float_array}")
        # Crear un publicador para el tipo de mensaje Float32MultiArray
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pose', 10)
        msg = Float32MultiArray()
        msg.data = self.float_array
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FloatArrayPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Destruir el nodo explícitamente
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
