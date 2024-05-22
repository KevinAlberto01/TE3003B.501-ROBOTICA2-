import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Pose

class MyNode(Node):

    def __init__(self):
        super().__init__('path')
        #rclpy.spin_once(self)
        rclpy.get_default_context().on_shutdown(self.cleanup)

        ############################### TOPICO PATH CONFIGURATION ##############################
        self.declare_parameters(namespace='', parameters=[('float_array', rclpy.Parameter.Type.DOUBLE_ARRAY)])
        self.path_value = self.get_parameter('float_array').value
        self.get_logger().info(f"Obtained parameter: {self.path_value}")
        
        self.path_pub = self.create_publisher(Float32MultiArray, '/path', 10)
        
        self.path_msg = Float32MultiArray()
        self.path_msg.data = self.path_value
        self.path_pub.publish(self.path_msg)
        self.get_logger().info(f"Published: {self.path_msg.data}")

        ###############################         TOPICO POSE       ###################################
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        ###############################         TOPICO FLAG       ############################
        self.create_subscription(Int32, '/flag', self.flag_cb, 10)

        ###### CONSTANTS AND VARIABLES #########
        self.pose = Pose()
        self.x = 0.0  # x[m] Goal position
        self.y = 0.0  # y[m] Goal position

        self.position = 0
        self.recorrido = 0
        self.cb_status = 0
        self.i = 0

        while self.get_clock().now().nanoseconds == 0:
            self.get_logger().info("No simulated time has been received yet")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Got time path generator")

        now = self.get_clock().now()
        start_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        state = "Program Pose"
        self.get_logger().info(f"state: {state}")
        self.rate = self.create_rate(80)  # 80 Hz

        while rclpy.ok():
            rclpy.spin_once(self)
            self.get_logger().info("Valor de status: %s" % self.cb_status)
            if self.cb_status == 1 and self.recorrido < len(self.path_value): #CHECAR ESTA CONDICION 
                self.get_logger().info("Entro al if DE PATH GENERATOR")

                x = self.path_value[self.i] #self.path_value[self.i * 2]
                y = self.path_value[self.i + 1] #self.path_value[self.i * 2 + 1] 

                self.pose.position.x = x
                self.pose.position.y = y
                self.i += 1
                self.recorrido += 1
                self.cb_status = 0
                state = "recibio bandera"
                self.pose_pub.publish(self.pose)
                self.get_logger().info(f"state: {state}")
                self.get_logger().info("Valor de pose: %s" % self.pose)
            else:
                state = "no recibio bandera"

    def flag_cb(self, msg):
        self.cb_status = msg.data
        if self.recorrido > len(self.path_value) - 1: # if self.recorrido > len(self.path_msg.data) // 2 - 1: LINEA ORIGINAL
            self.get_logger().info("Proceso terminado")
        else:
            self.get_logger().info(f"Se recibió la bandera: {self.cb_status}")

    def cleanup(self):
        self.get_logger().info('Cleaning up before shutdown')
        self.i = 0
        self.recorrido = 0
        self.pose = Pose() #MODIFIQUE ESTO
        self.get_logger().info("Valor de pose: %s" % self.pose)
        self.pose_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
