import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Int32
from geometry_msgs.msg import Pose

class MyNode(Node):

    def __init__(self):
        super().__init__('path')
        rclpy.get_default_context().on_shutdown(self.cleanup)

        ############################### TOPICO PATH CONFIGURATION ##############################
        self.declare_parameters(namespace='', parameters=[('float_array', rclpy.Parameter.Type.DOUBLE_ARRAY)])        # Declarar el parámetro 'float_array' como una lista de doubles
        self.path_value = self.get_parameter('float_array').value        # Obtener el valor del parámetro 'float_array'
        self.get_logger().info(f"Obtained parameter: {self.path_value}") #MUESTRA ESTO
        self.path_pub= self.create_publisher(Float32MultiArray, 'path', 10) # Crear un publicador para el tipo de mensaje Float32MultiArray
        
        self.path_msg = Float32MultiArray()
        self.path_msg.data= self.path_value
        self.path_pub.publish(self.path_msg)
        self.get_logger().info(f"Published: {self.path_msg.data}")# MUESTRA ESTO

        ###############################         TOPICO POSE       ###################################
        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

        ###############################         TOPICO FLAG       ############################
        self.create_subscription(Int32, 'flag', self.flag_cb, 10)

        ###### CONSTANTS AND VARIABLES #########
        self.pose = Pose()
        self.x = 0.0  # x[m] Goal position
        self.y = 0.0  # y[m] Goal position

        self.position = 0
        self.recorrido = 0
        self.cb_status = 0 #MODIFICAR ESTE VALOR
        self.i = 0

        while self.get_clock().now().nanoseconds == 0:
            self.get_logger().info("No simulated time has been received yet")
            #rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Got time path generator") #MUESTRA ESTO

        # Obtener el tiempo actual en segundos flotantes
        now = self.get_clock().now()
        start_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        # Definir el estado
        state = "Program Pose" #MUESTRA ESTO
        # Imprimir el estado
        self.get_logger().info(f"state: {state}") #MUESTRA ESTO
        # Crear un objeto Rate para definir la frecuencia del bucle
        self.rate = self.create_rate(80)  # 80 Hz

        while rclpy.ok():
            #state = "rclpy ok" #MUESTRA ESTO
            # Imprimir el estado
            #self.get_logger().info(f"state: {state}")
            #self.get_logger().info("Valor de status: %s" % self.cb_status)
            if self.cb_status == 1 and self.recorrido < len(self.path_msg.data): #preguntarle a SAM
                self.get_logger().info("Entro al if")

                x = self.path_value[self.i * 2]       # Obtener la coordenada x
                y = self.path_value[self.i * 2 + 1]

                self.pose.position.x = x
                self.pose.position.y = y
                self.i += 1
                self.recorrido += 1
                self.cb_status = 0
                state = "recibio banera" #MUESTRA ESTO
                self.pose_pub.publish(self.pose)
                self.get_logger().info(f"state: {state}")
                self.get_logger().info("Valor de pose: %s" % self.pose)
            else:
                state = "no recibio banera" #MUESTRA ESTO
                # Imprimir el estado
                #self.get_logger().info(f"state: {state}")

    

    def flag_cb(self, msg):
        self.get_logger().info("Funcion de flag empezo")
        self.cb_status = msg.data
        if self.recorrido > len(self.path_msg.data) - 1:
            self.get_logger().info("Proceso terminado")
        else:
            self.get_logger().info(f"Se recibió la bandera: {self.cb_status}")

    def cleanup(self):
        self.get_logger().info('Cleaning up before shutdown')
        self.i = 0
        self.recorrido = 0
        self.pose = Pose()
        self.get_logger().info("Valor de pose: %s" % self.pose)
        self.pose_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
